#!/usr/bin/env python
"""BWSC 2027 regulation checker for STEP models.

Reads a STEP file (assembly or single body), finds the wheels and the ground plane,
and checks the geometric rules that can be measured from CAD:

  2.2.1  envelope 5800 x 2300 x 1650 mm (tyre contact points on the base)
  2.3.1  at least three wheels
  2.3.2  front overhang < 60 % of wheelbase
  2.3.3  rear overhang  < 60 % of wheelbase
  2.3.4  road apex 1:15 up / 1:15 down, nothing but tyres touches
  2.3.5  speed hump 75 mm high (150 + 150 + 150 mm), nothing but tyres touches
  2.3.6  approach and departure angles >= 10 deg
  2.4.2  solar collector area <= 6 m2 (Challenger), from parts named like solar/cell/PV
  2.19.4 static stability factor SSF > 1 (needs the laden centre of mass, --cg)

Everything else (occupant cell, PVC Pat, head space, rear vision, lamps ...) is
listed in the report as a manual check.

Usage:
  python regcheck.py car.step
  python regcheck.py car.step --cg 2300,0,420 --out report.md
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

RULES_FILE = Path(__file__).with_name("rules_bwsc2027.json")
WHEEL_NAME = re.compile(r"wheel|tire|tyre|rim|휠|타이어|바퀴", re.I)
SOLAR_NAME = re.compile(r"solar|cell|pv|array|셀|태양", re.I)
AXES = {"x": 0, "y": 1, "z": 2}


# --------------------------------------------------------------------------- reading
@dataclass
class Part:
    name: str
    shape: object
    bmin: np.ndarray = None
    bmax: np.ndarray = None
    points: np.ndarray = None
    tris: np.ndarray = None
    area: float = 0.0
    is_wheel: bool = False
    is_solar: bool = False


def _label_name(label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def read_step(path: Path) -> list[Part]:
    """Read a STEP file into named solids. Falls back to an unnamed read."""
    parts: list[Part] = []
    try:
        doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        if reader.ReadFile(str(path)) != IFSelect_RetDone:
            raise RuntimeError("STEP read failed")
        reader.Transfer(doc)
        st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
        free = TDF_LabelSequence()
        st.GetFreeShapes(free)

        def walk(label, loc, names):
            name = _label_name(label)
            if XCAFDoc_ShapeTool.IsAssembly_s(label):
                comps = TDF_LabelSequence()
                XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
                for i in range(1, comps.Length() + 1):
                    comp = comps.Value(i)
                    ref = TDF_Label()
                    if XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref):
                        cloc = XCAFDoc_ShapeTool.GetLocation_s(comp)
                        walk(ref, loc.Multiplied(cloc), names + [_label_name(comp) or name])
            else:
                shape = XCAFDoc_ShapeTool.GetShape_s(label).Moved(loc)
                seq = []
                for n in names + [name]:
                    if n and (not seq or seq[-1] != n):
                        seq.append(n)
                full = " / ".join(seq)
                _add_solids(parts, shape, full)

        for i in range(1, free.Length() + 1):
            walk(free.Value(i), TopLoc_Location(), [])
    except Exception as exc:  # pragma: no cover - fallback path
        print(f"[info] named read failed ({exc}); reading geometry only", file=sys.stderr)
        parts = []
    if not parts:
        reader = STEPControl_Reader()
        if reader.ReadFile(str(path)) != IFSelect_RetDone:
            sys.exit(f"cannot read {path}")
        reader.TransferRoots()
        _add_solids(parts, reader.OneShape(), "")
    return parts


def _add_solids(parts: list[Part], shape, name: str):
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    found = False
    k = 0
    while exp.More():
        found = True
        k += 1
        parts.append(Part(name or f"solid_{len(parts) + 1}", exp.Current()))
        exp.Next()
    if not found:  # surfaces only (e.g. body skin exported as faces)
        parts.append(Part(name or f"surface_{len(parts) + 1}", shape))


def mesh_part(p: Part, tol: float):
    BRepMesh_IncrementalMesh(p.shape, tol, False, 0.5, True)
    box = Bnd_Box()
    BRepBndLib.Add_s(p.shape, box, True)
    if box.IsVoid():  # empty body in the STEP file
        p.points = np.zeros((0, 3))
        p.tris = np.zeros((0, 3), dtype=int)
        return False
    x0, y0, z0, x1, y1, z1 = box.Get()
    p.bmin, p.bmax = np.array([x0, y0, z0]), np.array([x1, y1, z1])
    pts, tris = [], []
    exp = TopExp_Explorer(p.shape, TopAbs_FACE)
    offset = 0
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            for i in range(1, n + 1):
                q = tri.Node(i).Transformed(trsf)
                pts.append((q.X(), q.Y(), q.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    b, c = c, b
                tris.append((offset + a - 1, offset + b - 1, offset + c - 1))
            offset += n
        exp.Next()
    p.points = np.array(pts) if pts else np.zeros((0, 3))
    p.tris = np.array(tris, dtype=int) if tris else np.zeros((0, 3), dtype=int)
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(p.shape, props)
    p.area = props.Mass()
    return True


def dense_points(p: Part, step: float, cap: int = 3_000_000) -> np.ndarray:
    """Vertices of the mesh after splitting triangles until every edge < step.

    Mesh vertices alone miss the inside of large flat faces (a box bottom has only
    4 corners), so clearance checks sample the faces themselves."""
    if not len(p.tris):
        return p.points
    tri = p.points[p.tris]  # (n, 3, 3)
    done = []
    total = 0
    for _ in range(12):
        e = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
        small = e <= step
        done.append(tri[small])
        total += int(small.sum())
        tri = tri[~small]
        if not len(tri) or total + 4 * len(tri) > cap:
            break
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        tri = np.concatenate([np.stack(t, axis=1) for t in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca))])
    done.append(tri)
    return np.concatenate([t.reshape(-1, 3) for t in done])


# --------------------------------------------------------------------------- frame
@dataclass
class Frame:
    long: int
    lat: int
    up: int
    up_sign: float
    fwd_sign: float

    def to_local(self, pts: np.ndarray) -> np.ndarray:
        """Return (x forward, y lateral, z up) coordinates."""
        return np.stack([pts[:, self.long] * self.fwd_sign, pts[:, self.lat], pts[:, self.up] * self.up_sign], axis=1)


def parse_axis(s: str):
    s = s.strip().lower()
    sign = -1.0 if s.startswith("-") else 1.0
    return AXES[s.lstrip("+-")], sign


def guess_frame(parts: list[Part], up: str | None, fwd: str | None) -> Frame:
    allmin = np.min([p.bmin for p in parts], axis=0)
    allmax = np.max([p.bmax for p in parts], axis=0)
    ext = allmax - allmin
    order = np.argsort(ext)  # small .. large : height, width, length (solar cars)
    up_i, up_s = parse_axis(up) if up else (int(order[0]), 0.0)
    long_i, f_s = parse_axis(fwd) if fwd else (int(order[2]) if int(order[2]) != up_i else int(order[1]), 1.0)
    lat_i = 3 - up_i - long_i
    if up_s == 0.0:  # pick the side the wheels touch
        score = {}
        for s in (1.0, -1.0):
            lo = (allmin if s > 0 else -allmax)[up_i]
            score[s] = sum(1 for p in parts if abs((p.bmin if s > 0 else -p.bmax)[up_i] - lo) < 0.02 * ext[up_i] and _wheel_shape(p, up_i, long_i, lat_i))
        up_s = 1.0 if score[1.0] >= score[-1.0] else -1.0
    return Frame(long_i, lat_i, up_i, up_s, f_s)


def _wheel_shape(p: Part, up_i, long_i, lat_i) -> bool:
    e = p.bmax - p.bmin
    d = max(e[up_i], e[long_i])
    if not (250 <= d <= 900):
        return False
    return 0.8 <= e[up_i] / max(e[long_i], 1e-9) <= 1.25 and e[lat_i] < 0.7 * d


# --------------------------------------------------------------------------- analysis
@dataclass
class Wheel:
    x: float
    y: float
    d: float
    names: list = field(default_factory=list)


def find_wheels(parts: list[Part], fr: Frame, ground: float, height: float, name_re) -> list[Wheel]:
    cands = []
    for p in parts:
        named = bool(name_re.search(p.name))
        loc =fr.to_local(np.array([p.bmin, p.bmax]))
        lo_z = loc[:, 2].min()
        if abs(lo_z - ground) > max(5.0, 0.01 * height):
            continue
        if named or _wheel_shape(p, fr.up, fr.long, fr.lat):
            p.is_wheel = True
            c = loc.mean(axis=0)
            d = loc[:, 2].max() - lo_z
            cands.append(Wheel(c[0], c[1], d, [p.name]))
    wheels: list[Wheel] = []
    for c in cands:  # merge tyre + rim + hub pieces sharing a centre
        for w in wheels:
            if math.hypot(w.x - c.x, w.y - c.y) < 60:
                w.d = max(w.d, c.d)
                w.names += c.names
                break
        else:
            wheels.append(c)
    return wheels


def convex_hull_2d(pts):
    pts = sorted(set(map(tuple, pts)))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def ssf(contacts, cg_xy, cg_h):
    hull = convex_hull_2d(contacts)
    if len(hull) < 3:
        return None, None
    best = None
    for i in range(len(hull)):
        a, b = np.array(hull[i]), np.array(hull[(i + 1) % len(hull)])
        ab = b - a
        dist = abs(ab[0] * (cg_xy[1] - a[1]) - ab[1] * (cg_xy[0] - a[0])) / np.linalg.norm(ab)
        best = dist if best is None else min(best, dist)
    return (best / cg_h if cg_h else None), best


def hump_profile(x, lim):
    """Height of the 2.3.5 speed hump at distance x from its start (worst case: longest top)."""
    a, top, h = lim["hump_ramp_mm"], lim["hump_top_max_mm"], lim["hump_height_mm"]
    x = np.asarray(x, dtype=float)
    return np.clip(np.minimum(x, a + top + a - x) / a, 0.0, 1.0) * h


def hump_clearance(body, ground, xr, xf, r_rear, r_front, lim, step=10.0):
    """Drive the car over the speed hump in side view and return the smallest gap.

    Wheels are circles rolling over the hump; the body pitches about the two axles.
    Returns (min margin mm, body x of that point, hump start x)."""
    total = 2 * lim["hump_ramp_mm"] + lim["hump_top_max_mm"]
    wb = xf - xr
    # lower envelope of the body in 10 mm slices along the car
    idx = np.floor((body[:, 0] - body[:, 0].min()) / step).astype(int)
    zb = np.full(idx.max() + 1, np.inf)
    np.minimum.at(zb, idx, body[:, 2] - ground)
    ok = np.isfinite(zb)
    xb = (body[:, 0].min() + (np.arange(len(zb)) + 0.5) * step)[ok]
    zb = zb[ok]
    starts = np.arange(body[:, 0].min() - total - 700, body[:, 0].max() + 700, step)

    def lift(xw, r):
        if not np.isfinite(r) or r <= 0:  # contact points only: follow the hump surface
            return hump_profile(xw - starts, lim)
        u = np.linspace(-r, r, 81)
        z = hump_profile(xw + u[None, :] - starts[:, None], lim) + np.sqrt(np.clip(r * r - u * u, 0, None))[None, :]
        return np.clip(z.max(axis=1) - r, 0, None)

    dr, df = lift(xr, r_rear), lift(xf, r_front)
    raise_ = dr[:, None] + (df - dr)[:, None] * (xb[None, :] - xr) / wb
    margin = zb[None, :] + raise_ - hump_profile(xb[None, :] - starts[:, None], lim)
    i, j = np.unravel_index(np.argmin(margin), margin.shape)
    return float(margin[i, j]), float(xb[j]), float(starts[i])


def save_plot(path, res, body, wheels, ground, lim, cg=None):
    """Side and top view with the envelope, clearance lines and support polygon."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MplPolygon, Rectangle

    pts = body[:: max(1, len(body) // 60000)]
    x0, x1 = body[:, 0].min(), body[:, 0].max()
    for w in wheels:
        r = w.d / 2 if np.isfinite(w.d) else 0.0
        x0, x1 = min(x0, w.x - r), max(x1, w.x + r)
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(13, 9), gridspec_kw={"height_ratios": [1, 1.25]})
    ax.scatter(pts[:, 0], pts[:, 2] - ground, s=0.3, c="#7a7a7a", linewidths=0)
    for w in wheels:
        if np.isfinite(w.d):
            ax.add_patch(Circle((w.x, w.d / 2), w.d / 2, fill=False, ec="#1f3864", lw=1.2))
    ax.axhline(0, c="k", lw=1)
    cx = (x0 + x1) / 2
    ax.add_patch(Rectangle((cx - lim["length_mm"] / 2, 0), lim["length_mm"], lim["height_mm"], fill=False, ec="#c00000", ls="--", lw=1,
                           label=f"2.2.1 envelope {lim['length_mm']} x {lim['height_mm']} mm"))
    if len(wheels) >= 2:
        xs = [w.x for w in wheels]
        xf, xr = max(xs), min(xs)
        wb = xf - xr
        reach = 0.7 * wb
        ax.plot([xf, xf + reach], [0, reach * math.tan(math.radians(lim["approach_deg"]))], c="#2e75b6", lw=1,
                label=f"2.3.6 approach / departure {lim['approach_deg']} deg")
        ax.plot([xr, xr - reach], [0, reach * math.tan(math.radians(lim["departure_deg"]))], c="#2e75b6", lw=1)
        b = np.linspace(0, wb, 100)
        ax.plot(xr + b, 2 * b * (wb - b) / (lim["apex_grade"] * wb), c="#ed7d31", lw=1.2, label=f"2.3.4 road apex 1:{lim['apex_grade']:.0f}")
        if res.get("hump"):
            s = res["hump"]["start"]
            a, top = lim["hump_ramp_mm"], lim["hump_top_max_mm"]
            ax.fill([s, s + a, s + a + top, s + 2 * a + top], [0, lim["hump_height_mm"], lim["hump_height_mm"], 0],
                    c="#70ad47", alpha=0.7, label="2.3.5 speed hump (tightest position)")
    ax.set_aspect("equal")
    ax.set_ylim(-80, lim["height_mm"] + 150)
    ax.set_title(f"Side view - {Path(res['file']).name}")
    ax.set_xlabel("length (mm)")
    ax.set_ylabel("height above ground (mm)")
    ax.legend(loc="upper right", fontsize=8)

    bx.scatter(pts[:, 0], pts[:, 1], s=0.3, c="#7a7a7a", linewidths=0)
    cy = (body[:, 1].min() + body[:, 1].max()) / 2
    bx.add_patch(Rectangle((cx - lim["length_mm"] / 2, cy - lim["width_mm"] / 2), lim["length_mm"], lim["width_mm"], fill=False, ec="#c00000",
                           ls="--", lw=1, label=f"2.2.1 envelope {lim['length_mm']} x {lim['width_mm']} mm"))
    if wheels:
        c = [(w.x, w.y) for w in wheels]
        hull = convex_hull_2d(c)
        if len(hull) >= 3:
            bx.add_patch(MplPolygon(hull, closed=True, fill=True, fc="#1f3864", alpha=0.12, ec="#1f3864", lw=1.2,
                                    label="tyre contact polygon (2.19.4 SSF)"))
        bx.scatter([q[0] for q in c], [q[1] for q in c], c="#1f3864", s=30, zorder=3)
    if cg is not None:
        bx.scatter([cg[0]], [cg[1]], marker="x", c="#c00000", s=70, zorder=4, label=f"centre of mass (h = {cg[2] - ground:.0f} mm)")
    bx.set_aspect("equal")
    bx.set_title("Top view")
    bx.set_xlabel("length (mm)")
    bx.set_ylabel("width (mm)")
    bx.legend(loc="upper right", fontsize=8)
    n = {k: sum(1 for c_ in res["checks"] if c_["result"] == k) for k in ("PASS", "FAIL", "CHECK")}
    fig.suptitle(f"BWSC 2027 regulation check: {n['PASS']} pass, {n['FAIL']} fail, {n['CHECK']} manual", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def run(path: Path, args) -> dict:
    rules = json.loads(RULES_FILE.read_text(encoding="utf-8"))
    lim = rules["limits"]
    parts = read_step(path)
    parts = [p for p in parts if mesh_part(p, args.mesh_tol)]
    fr = guess_frame(parts, args.up, args.forward)
    pts_all = [fr.to_local(p.points) for p in parts if len(p.points)]
    allpts = np.vstack(pts_all)
    ground = allpts[:, 2].min()
    height = allpts[:, 2].max() - ground
    wheel_re = re.compile(args.wheel_name, re.I) if args.wheel_name else WHEEL_NAME
    wheels = find_wheels(parts, fr, ground, height, wheel_re)
    if args.contacts:  # body-only models: tyre contact points given by hand (model mm)
        for p in parts:
            p.is_wheel = False
        wheels = []
        for k, xy in enumerate(args.contacts.split(";"), 1):
            v = [float(t) for t in xy.split(",")]
            q = np.zeros(3)
            q[fr.long], q[fr.lat] = v[0], v[1]
            loc = fr.to_local(q[None, :])[0]
            wheels.append(Wheel(loc[0], loc[1], float("nan"), [f"--contacts #{k}"]))
        if args.ground is not None:
            ground = float(args.ground) * fr.up_sign
            height = allpts[:, 2].max() - ground
    body = np.vstack([fr.to_local(dense_points(p, args.sample)) for p in parts if len(p.points) and not p.is_wheel] or [np.zeros((0, 3))])

    res = {"file": str(path), "frame": {"length_axis": "xyz"[fr.long], "up_axis": ("+" if fr.up_sign > 0 else "-") + "xyz"[fr.up],
                                        "forward": ("+" if fr.fwd_sign > 0 else "-") + "xyz"[fr.long]},
           "parts": len(parts), "wheels": [w.__dict__ for w in wheels], "checks": []}

    def add(rule, item, limit, value, ok, note=""):
        ok = None if ok is None else bool(ok)
        res["checks"].append({"rule": rule, "item": item, "limit": limit, "value": value,
                              "result": "PASS" if ok is True else ("FAIL" if ok is False else "CHECK"), "note": note})

    L = allpts[:, 0].max() - allpts[:, 0].min()
    W = allpts[:, 1].max() - allpts[:, 1].min()
    add("2.2.1", "전장 (length)", f"≤ {lim['length_mm']} mm", f"{L:.0f} mm", L <= lim["length_mm"])
    add("2.2.1", "전폭 (width)", f"≤ {lim['width_mm']} mm", f"{W:.0f} mm", W <= lim["width_mm"])
    add("2.2.1", "전고 (height, 타이어 접지면 기준)", f"≤ {lim['height_mm']} mm", f"{height:.0f} mm", height <= lim["height_mm"])
    add("2.3.1", "바퀴 수 (wheels)", f"≥ {lim['min_wheels']}", f"{len(wheels)}", len(wheels) >= lim["min_wheels"],
        "" if wheels else "바퀴를 못 찾음: --wheel-name 으로 바퀴 부품 이름을 지정하세요")

    if len(wheels) >= 2:
        xs = [w.x for w in wheels]
        xf, xr = max(xs), min(xs)
        wb = xf - xr
        front_oh = allpts[:, 0].max() - xf
        rear_oh = xr - allpts[:, 0].min()
        add("2.3.2", "앞 오버행 / 휠베이스", f"< {lim['overhang_ratio']:.0%}", f"{front_oh:.0f} / {wb:.0f} mm = {front_oh / wb:.2%}", front_oh < lim["overhang_ratio"] * wb)
        add("2.3.3", "뒤 오버행 / 휠베이스", f"< {lim['overhang_ratio']:.0%}", f"{rear_oh:.0f} / {wb:.0f} mm = {rear_oh / wb:.2%}", rear_oh < lim["overhang_ratio"] * wb)

        tan_a = math.tan(math.radians(lim["approach_deg"]))
        fr_pts = body[body[:, 0] > xf + 1]
        if len(fr_pts):
            ang = np.degrees(np.arctan2(fr_pts[:, 2] - ground, fr_pts[:, 0] - xf)).min()
            add("2.3.6", "접근각 (approach)", f"≥ {lim['approach_deg']}°", f"{ang:.1f}°", ang >= lim["approach_deg"])
        rr_pts = body[body[:, 0] < xr - 1]
        if len(rr_pts):
            ang = np.degrees(np.arctan2(rr_pts[:, 2] - ground, xr - rr_pts[:, 0])).min()
            add("2.3.6", "이탈각 (departure)", f"≥ {lim['departure_deg']}°", f"{ang:.1f}°", ang >= lim["departure_deg"])

        m = 1.0 / lim["apex_grade"]
        mid = body[(body[:, 0] > xr) & (body[:, 0] < xf)]
        if len(mid):
            b = mid[:, 0] - xr
            need = 2 * m * b * (wb - b) / wb
            margin = (mid[:, 2] - ground) - need
            i = int(np.argmin(margin))
            add("2.3.4", f"도로 정점 1:{lim['apex_grade']:.0f} 통과 (최소 여유)", "> 0 mm",
                f"{margin[i]:.0f} mm (뒷바퀴에서 {b[i]:.0f} mm 지점, 지상고 {mid[i, 2] - ground:.0f} mm)", margin[i] > 0,
                f"휠베이스 중앙에서 필요한 지상고 {wb * m / 2:.0f} mm")

        rad = lambda x_: max([w.d for w in wheels if abs(w.x - x_) < 60 and np.isfinite(w.d)] or [float("nan")]) / 2
        if len(body):
            hm, hx_, hs = hump_clearance(body, ground, xr, xf, rad(xr), rad(xf), lim)
            res["hump"] = {"margin": hm, "x": hx_, "start": hs}
            add("2.3.5", f"과속방지턱 통과 (높이 {lim['hump_height_mm']} mm, 최소 여유)", "> 0 mm",
                f"{hm:.0f} mm (뒷바퀴에서 {hx_ - xr:.0f} mm 지점)", hm > 0,
                f"경사 {lim['hump_ramp_mm']} + 윗면 {lim['hump_top_max_mm']} + 경사 {lim['hump_ramp_mm']} mm. 바퀴가 턱을 넘을 때 차체가 기우는 것까지 계산")

        contacts = [(w.x, w.y) for w in wheels]
        if args.cg:
            cg = np.array([float(v) for v in args.cg.split(",")])
            cg_l = fr.to_local(cg[None, :])[0]
            val, dist = ssf(contacts, cg_l[:2], cg_l[2] - ground)
            if val is not None:
                add("2.19.4", "정적 안정성 계수 SSF", f"> {lim['ssf']}", f"{val:.2f} (지지선까지 {dist:.0f} mm / 무게중심 높이 {cg_l[2] - ground:.0f} mm)", val > lim["ssf"])
        else:
            hull = np.array(convex_hull_2d(contacts))
            if len(hull) >= 3:
                c = hull.mean(axis=0)
                _, dist = ssf(contacts, c, 1.0)
                add("2.19.4", "정적 안정성 계수 SSF", f"> {lim['ssf']}", "무게중심 미입력", None,
                    f"무게중심이 바퀴 배치 중앙에 있다면 높이 {dist:.0f} mm 미만이어야 SSF>1. --cg x,y,z 로 만재 무게중심을 넣으면 판정합니다")

    solar_re = re.compile(args.solar_name, re.I) if args.solar_name else SOLAR_NAME
    solar = [p for p in parts if solar_re.search(p.name) and not p.is_wheel]
    if solar:
        items = []  # (projected up-facing area, plan bbox) per part
        for p in solar:
            if len(p.tris):
                v = fr.to_local(p.points)
                t = p.tris
                n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]) / 2
                a = n[n[:, 2] > 0, 2].sum()
                items.append((a, v[:, :2].min(axis=0), v[:, :2].max(axis=0)))
        kept = []  # stacked layers (cell + encapsulant + glass) share a footprint: count once
        for a, lo, hi in sorted(items, key=lambda x: -x[0]):
            for _, klo, khi in kept:
                ix = np.clip(np.minimum(hi, khi) - np.maximum(lo, klo), 0, None).prod()
                if ix > 0.8 * min((hi - lo).prod(), (khi - klo).prod()):
                    break
            else:
                kept.append((a, lo, hi))
        area_m2 = sum(k[0] for k in kept) / 1e6
        add("2.4.2", "태양광 집광 면적 (위에서 본 투영)", f"≤ {lim['solar_area_m2']} m²", f"{area_m2:.2f} m² ({len(kept)}개 부품, 겹친 층 {len(items) - len(kept)}개 제외)",
            area_m2 <= lim["solar_area_m2"], "부품 이름이 solar/cell/PV/셀/태양 인 것만 합산. 곡면 실면적은 투영보다 큼")
    else:
        add("2.4.2", "태양광 집광 면적", f"≤ {lim['solar_area_m2']} m²", "해당 부품 없음", None,
            "--solar-name 으로 셀 부품 이름을 지정하거나 셀 배치 STEP을 따로 넣으세요")

    for r in rules["manual"]:
        add(r["rule"], r["item"], r["limit"], "-", None, "CAD 자동 판정 대상 아님")
    if getattr(args, "plot", None):
        cg_l = fr.to_local(np.array([[float(v) for v in args.cg.split(",")]]))[0] if args.cg else None
        save_plot(args.plot, res, body if len(body) else allpts, wheels, ground, lim, cg_l)
    return res


# --------------------------------------------------------------------------- report
def to_markdown(res: dict) -> str:
    mark = {"PASS": "✅ 통과", "FAIL": "❌ 불합격", "CHECK": "⚠️ 수동 확인"}
    lines = [f"# BWSC 2027 규정 체크 – {Path(res['file']).name}", "",
             f"- 좌표: 길이축 {res['frame']['length_axis']}, 위쪽 {res['frame']['up_axis']}, 앞쪽 {res['frame']['forward']}",
             f"- 부품 {res['parts']}개, 바퀴 {len(res['wheels'])}개 인식", ""]
    if res["wheels"]:
        lines += ["| 바퀴 | 길이방향 | 좌우 | 지름 | 부품 |", "|---|---|---|---|---|"]
        for i, w in enumerate(sorted(res["wheels"], key=lambda w: -w["x"]), 1):
            names = sorted({' / '.join(s for s in n.split(' / ') if not re.fullmatch(r'NAUO\d+', s)) for n in w['names']})
            lines.append(f"| {i} | {w['x']:.0f} | {w['y']:.0f} | {w['d']:.0f} | {', '.join(names)[:80]} |")
        lines.append("")
    n = {k: sum(1 for c in res["checks"] if c["result"] == k) for k in ("PASS", "FAIL", "CHECK")}
    lines += [f"**통과 {n['PASS']} · 불합격 {n['FAIL']} · 수동 확인 {n['CHECK']}**", "",
              "| 조항 | 항목 | 기준 | 측정값 | 판정 | 비고 |", "|---|---|---|---|---|---|"]
    for c in res["checks"]:
        lines.append(f"| {c['rule']} | {c['item']} | {c['limit']} | {c['value']} | {mark[c['result']]} | {c['note']} |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description="BWSC 2027 regulation check for STEP models")
    ap.add_argument("step", type=Path)
    ap.add_argument("--up", help="up axis, e.g. z, -y (default: auto)")
    ap.add_argument("--forward", help="forward axis, e.g. x, -x (default: longest axis, +)")
    ap.add_argument("--cg", help="laden centre of mass x,y,z in model mm (for SSF)")
    ap.add_argument("--wheel-name", help="regex for wheel part names")
    ap.add_argument("--contacts", help='tyre contact points "x,y;x,y;x,y" (model mm, length/lateral axes) for body-only models')
    ap.add_argument("--ground", type=float, help="ground level along the up axis (model mm) when using --contacts")
    ap.add_argument("--solar-name", help="regex for solar cell part names")
    ap.add_argument("--mesh-tol", type=float, default=2.0, help="mesh deflection in mm (default 2)")
    ap.add_argument("--sample", type=float, default=50.0, help="max spacing of clearance sample points in mm (default 50)")
    ap.add_argument("--plot", type=Path, help="write a side/top view PNG here (needs matplotlib)")
    ap.add_argument("--out", type=Path, help="write markdown report here")
    ap.add_argument("--json", type=Path, help="write JSON result here")
    a = ap.parse_args(argv)
    res = run(a.step, a)
    md = to_markdown(res)
    print(md)
    if a.out:
        a.out.write_text(md, encoding="utf-8")
    if a.json:
        a.json.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    return 1 if any(c["result"] == "FAIL" for c in res["checks"]) else 0


if __name__ == "__main__":
    sys.exit(main())
