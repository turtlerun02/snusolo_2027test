"""Build a simple 4-wheel test car (box body + 4 wheels + flat solar panel) as STEP.

    python examples/make_sample.py          -> examples/sample_car.step
"""
from pathlib import Path

import cadquery as cq

L, W, H = 4500.0, 1700.0, 900.0     # body box (mm)
CLEAR = 200.0                        # body underside above ground
D, T = 560.0, 90.0                   # wheel diameter, width
WB, TRACK = 2700.0, 1300.0           # wheelbase, track
X0 = 1300.0                          # rear axle x (front axle = X0 + WB)

asm = cq.Assembly(name="sample_car")
body = cq.Workplane("XY").box(L, W, H, centered=(False, True, False)).translate((0, 0, CLEAR))
asm.add(body, name="body_shell")
for i, x in enumerate((X0, X0 + WB)):
    for j, y in enumerate((-TRACK / 2, TRACK / 2)):
        wheel = cq.Workplane("XZ").circle(D / 2).extrude(T / 2, both=True).translate((x, y, D / 2))
        asm.add(wheel, name=f"wheel_{i}{j}")
panel = cq.Workplane("XY").box(3000, 1500, 5, centered=(False, True, False)).translate((700, 0, CLEAR + H))
asm.add(panel, name="solar_array")

out = Path(__file__).with_name("sample_car.step")
asm.save(str(out))
print("wrote", out)
