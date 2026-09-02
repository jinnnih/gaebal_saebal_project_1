# meshes

`valet_car` 외형 메시.

| 파일 | 내용 |
|---|---|
| `body.obj` | 차체 + 실내 + 유리 (1000 정점 / 1680 면) |
| `wheel.obj` | 바퀴 1개. 원점 중심, 회전축 = y (URDF 바퀴 규약) |
| `valet_car.mtl` | 재질 |
| `textures/` | Hybrid.png, Hybrid_Interior.png, Wheels3.png |

## 출처와 라이선스

원본은 Gazebo Fuel 의 **OpenRobotics / Prius Hybrid**, 저자 Ian Chen,
라이선스 **CC0 1.0 Universal (퍼블릭 도메인)**.
https://app.gazebosim.org/OpenRobotics/fuel/models/Prius%20Hybrid

CC0 라 별도 표기 의무는 없지만 출처를 남긴다.

## 어떻게 변형했나

원본 Prius 와 valet_car 는 제원이 다르다.

| | 전장 | 전폭 | 축거 | 바퀴반경 |
|---|---|---|---|---|
| Prius | 4.618 | 2.012 | 2.865 | 0.305 |
| valet_car | 4.50 | 1.90 | **2.50** | 0.33 |

균일 스케일로는 축거가 안 맞아 바퀴가 휠아치에서 15 cm 어긋난다.
그래서 `tools/fit_car_mesh.py` 가 길이축을 **구간별 선형 리매핑** 한다.

```
 앞 오버행  x1.170     축간 x0.873     뒤 오버행 x1.112
 폭 x0.944            높이 x0.794
```

높이를 0.794 로 줄인 이유는 지붕이 **1.34 m** 여야 하기 때문이다.
라이다가 1.42 m 에 있고, 그보다 지붕이 낮아야 360도가 안 가린다.
(자세한 근거는 `urdf/common.xacro` 3장)

## 재생성

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid"
python3 tools/fit_car_mesh.py \
  ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/prius\ hybrid/3/meshes/Hybrid.obj \
  src/valet_robot/meshes/
```
