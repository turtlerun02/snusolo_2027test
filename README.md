# bwsc-reg-check

STEP 파일로 **2027 Bridgestone World Solar Challenge(BWSC)** 차량 규정 중 CAD로 잴 수 있는 항목을 자동으로 확인하는 도구입니다.
Checks the CAD-measurable rules of the 2027 BWSC Event Regulations from a STEP model.

## 자동 판정 항목

| 조항 | 항목 | 기준 |
|---|---|---|
| 2.2.1 | 전장 / 전폭 / 전고 (타이어 접지면 기준) | 5800 × 2300 × 1650 mm 이내 |
| 2.3.1 | 바퀴 수 | 3개 이상 |
| 2.3.2 / 2.3.3 | 앞·뒤 오버행 | 휠베이스의 60 % 미만 |
| 2.3.4 | 도로 정점 (1:15 오르막 → 1:15 내리막) | 타이어 외에 닿는 곳 없음 |
| 2.3.6 | 접근각 / 이탈각 | 10° 이상 |
| 2.4.2 | 태양광 집광 면적 (Challenger) | 6 m² 이하 |
| 2.19.4 | 정적 안정성 계수 SSF (45° 기울임) | SSF > 1 (만재 무게중심 입력 시) |

탑승 셀(헬멧 50 mm), PVC Pat, 헤드 스페이스, 후방 시야, 등화 위치 같은 항목은 보고서에 **수동 확인**으로 표시됩니다.

## 설치

```bash
pip install -r requirements.txt
```

`cadquery-ocp`(OpenCascade 파이썬 바인딩)와 `numpy`가 필요합니다. 샘플 차량을 만들려면 `cadquery`도 설치하세요.

## 사용법

```bash
python regcheck.py car.step
python regcheck.py car.step --cg 2300,0,420 --out report.md --json report.json
```

| 옵션 | 설명 |
|---|---|
| `--cg x,y,z` | 만재 무게중심 (모델 좌표, mm). 넣으면 SSF를 판정합니다 |
| `--up z` / `--forward -x` | 위쪽 축, 앞쪽 축. 기본값은 자동 (가장 짧은 축 = 높이, 가장 긴 축 = 길이) |
| `--wheel-name REGEX` | 바퀴 부품 이름 패턴. 기본값 `wheel\|tire\|tyre\|rim\|휠\|타이어\|바퀴` + 모양 인식 |
| `--solar-name REGEX` | 태양광 셀 부품 이름 패턴. 기본값 `solar\|cell\|pv\|array\|셀\|태양` |
| `--contacts "x,y;x,y;x,y"` | 바퀴가 없는 바디 STEP일 때 타이어 접지점을 직접 입력 (`--ground`와 함께) |
| `--mesh-tol`, `--sample` | 메쉬 정밀도, 지상고 검사 점 간격 (mm) |

종료 코드: 불합격 항목이 있으면 1, 없으면 0 (CI에서 쓸 수 있음).

## 동작 방식

1. STEP을 OpenCascade로 읽고, 어셈블리 이름을 유지한 채 솔리드별로 메쉬를 만듭니다.
2. 가장 낮은 점을 지면으로 보고, 지면에 닿는 원형 부품(지름 250–900 mm) 또는 이름이 바퀴인 부품을 바퀴로 인식합니다.
3. 바퀴 접지점으로 휠베이스·오버행·지지 다각형을 구하고, 바퀴를 뺀 모든 면에서 촘촘히 뽑은 점으로 지상고·접근각·이탈각·도로 정점을 검사합니다.
   - 도로 정점: 뒷바퀴에서 거리 b인 지점은 지상고가 `2·b·(L−b) / (15·L)` 보다 커야 합니다 (L = 휠베이스).
4. 태양광 면적은 셀 부품의 위를 향한 면을 위에서 본 투영 면적의 합입니다. 같은 자리에 겹친 층(셀·봉지재·유리)은 한 번만 셉니다.
5. SSF는 바퀴 접지점의 볼록 다각형 각 변까지의 수평 거리 ÷ 무게중심 높이의 최솟값입니다.

## 예제

```bash
python examples/make_sample.py
python regcheck.py examples/sample_car.step --cg 2650,0,600
```

샘플 차량은 이탈각 8.7°로 2.3.6에서 불합격이 나오도록 만들어져 있습니다.

## Fusion 360 (.f3d / .f3z)

`.f3d`, `.f3z`는 Autodesk 전용 형식이라 직접 읽지 않습니다. Fusion에서 **파일 → 내보내기 → STEP**으로 저장한 뒤 사용하세요. 부품 이름이 STEP에 그대로 들어가므로 바퀴·셀 인식이 더 정확해집니다.

## 주의

- 이 도구는 설계 단계의 자가 점검용입니다. 공식 판정은 대회 검차(scrutineering)와 규정 원문을 따릅니다.
- 규정 수치는 `rules_bwsc2027.json`에 있습니다. 규정이 개정되면 이 파일만 고치면 됩니다.

## License

MIT
