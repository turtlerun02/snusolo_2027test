"""Checks the tool against the sample car, whose answers can be worked out by hand.

Sample car (examples/make_sample.py): body box 4500 x 1700 x 900 mm sitting 200 mm
above the ground, four 560 mm wheels, wheelbase 2700 mm, rear axle 1300 mm from the
tail, track 1300 mm, flat 3000 x 1500 mm solar panel.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import regcheck  # noqa: E402


def _args(**kw):
    base = dict(up=None, forward=None, cg=None, wheel_name=None, contacts=None, ground=None,
                solar_name=None, mesh_tol=2.0, sample=50.0, plot=None)
    base.update(kw)
    return argparse.Namespace(**base)


@pytest.fixture(scope="module")
def result():
    return regcheck.run(ROOT / "examples" / "sample_car.step", _args(cg="2650,0,600"))


def _check(result, rule, word=""):
    hits = [c for c in result["checks"] if c["rule"] == rule and word in c["item"]]
    assert len(hits) == 1, f"{rule} {word}: {len(hits)} rows"
    return hits[0]


def _num(text):
    return float(re.search(r"-?\d+(\.\d+)?", text.replace(",", "")).group())


def test_envelope(result):
    assert _num(_check(result, "2.2.1", "전장")["value"]) == pytest.approx(4500, abs=2)
    assert _num(_check(result, "2.2.1", "전폭")["value"]) == pytest.approx(1700, abs=2)
    assert _num(_check(result, "2.2.1", "전고")["value"]) == pytest.approx(1105, abs=3)


def test_wheels_found(result):
    assert len(result["wheels"]) == 4
    assert sorted(round(w["x"]) for w in result["wheels"]) == [1300, 1300, 4000, 4000]
    assert all(w["d"] == pytest.approx(560, abs=2) for w in result["wheels"])


def test_overhangs(result):
    front, rear = _check(result, "2.3.2"), _check(result, "2.3.3")
    assert "500 / 2700" in front["value"] and front["result"] == "PASS"
    assert "1300 / 2700" in rear["value"] and rear["result"] == "PASS"


def test_approach_and_departure(result):
    approach, departure = _check(result, "2.3.6", "접근"), _check(result, "2.3.6", "이탈")
    assert _num(approach["value"]) == pytest.approx(math.degrees(math.atan2(200, 500)), abs=0.3)
    assert approach["result"] == "PASS"
    # 200 mm clearance over a 1300 mm rear overhang is 8.7 degrees: below the 10 degree limit
    assert _num(departure["value"]) == pytest.approx(math.degrees(math.atan2(200, 1300)), abs=0.3)
    assert departure["result"] == "FAIL"


def test_road_apex(result):
    # needs 2700 / 15 / 2 = 90 mm at mid wheelbase, the floor is at 200 mm
    c = _check(result, "2.3.4")
    assert _num(c["value"]) == pytest.approx(110, abs=3) and c["result"] == "PASS"


def test_speed_hump(result):
    # flat floor at 200 mm over a 75 mm hump leaves 125 mm
    c = _check(result, "2.3.5")
    assert _num(c["value"]) == pytest.approx(125, abs=3) and c["result"] == "PASS"


def test_solar_area(result):
    c = _check(result, "2.4.2")
    assert _num(c["value"]) == pytest.approx(4.5, abs=0.02) and c["result"] == "PASS"


def test_ssf(result):
    # half track 650 mm / centre of mass height 600 mm
    c = _check(result, "2.19.4")
    assert _num(c["value"]) == pytest.approx(650 / 600, abs=0.02) and c["result"] == "PASS"


def test_hump_catches_a_low_floor():
    lim = dict(hump_ramp_mm=150, hump_top_max_mm=150, hump_height_mm=75)
    xs = np.arange(0, 3000, 10.0)
    floor = np.column_stack([xs, np.zeros_like(xs), np.full_like(xs, 60.0)])  # 60 mm floor
    margin, _, _ = regcheck.hump_clearance(floor, 0.0, 300.0, 2700.0, 280.0, 280.0, lim)
    assert margin == pytest.approx(60 - 75, abs=2)


def test_exit_code_and_report(tmp_path, capsys):
    out, png = tmp_path / "r.md", tmp_path / "r.png"
    code = regcheck.main([str(ROOT / "examples" / "sample_car.step"), "--out", str(out), "--plot", str(png)])
    capsys.readouterr()
    assert code == 1  # the sample fails the departure angle on purpose
    assert "2.3.6" in out.read_text(encoding="utf-8")
    assert png.stat().st_size > 10_000
