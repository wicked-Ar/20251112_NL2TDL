# MuJoCo 시뮬레이션 연동 가이드

## 📋 개요

NL2TDL Converter에 MuJoCo 시뮬레이션 연동 기능이 추가되었습니다!

이제 MuJoCo scene에서 객체 정보(위치, 질량, 방향)를 실시간으로 추출하고, 자연어 명령에서 언급된 객체를 scene에서 찾아 **실제 scene 데이터를 바탕으로 정확한 TDL 코드**를 생성할 수 있습니다.

---

## 🎯 주요 기능

### 1. 실시간 Scene 정보 추출
- MuJoCo scene의 모든 객체 파싱
- 위치, 회전, 질량, 크기 정보 추출
- 관절 각도 실시간 읽기

### 2. 지능형 객체 매칭
- 자연어 참조 ("용접기", "로봇 A") → 실제 객체 이름 매칭
- Fuzzy matching 알고리즘 적용
- 부분 문자열 및 유사도 기반 매칭

### 3. 정확한 TDL 생성
- Scene의 실제 위치/질량 정보 활용
- 자동 좌표 변환 (m → mm)
- 객체 간 거리 자동 계산

---

## 🚀 빠른 시작

### 1. 설치

```bash
# MuJoCo 라이브러리 설치
pip install mujoco numpy

# 또는 requirements.txt 사용
pip install -r requirements.txt
```

### 2. 실행

```bash
python main.py
```

메뉴에서 **4. MuJoCo Scene 연동 모드** 선택

### 3. Scene 파일 로드

```
MuJoCo XML 파일 경로: example_mujoco_scene.xml
```

예제 scene 파일이 제공됩니다!

### 4. 자연어 명령 입력

```
자연어 명령 (with scene)> 무게 15kg인 용접기를 로봇 A로 집어올려
```

---

## 📝 사용 예시

### 예시 1: 기본 사용

**Scene 정보:**
- `welding_tool_01`: 위치 (0.5m, 0.3m, 0.2m), 질량 15kg
- `robot_arm_A`: 현재 관절 각도 (0°, 0°, 90°, 0°, 90°, 0°)

**자연어 입력:**
```
무게 15kg인 용접기를 로봇으로 집어올려
```

**생성된 TDL (일부):**
```tdl
GOAL Initialize_Process()
{
    SPAWN SetWorkpieceWeight(15.0, PosX(0, 0, 0, 0, 0, 0)) WITH WAIT;
}

GOAL Execute_Process()
{
    SPAWN MoveLinear(PosX(500, 300, 200, 180, 0, 180), 60, 60, 0, 0) WITH WAIT;
    // Scene의 실제 위치 (0.5m → 500mm, 0.3m → 300mm, 0.2m → 200mm)
}
```

### 예시 2: 여러 객체 참조

**Scene 정보:**
- `workpiece_metal_plate`: 위치 (0.8m, 0m, 0.1m), 질량 5kg
- `tool_rack`: 위치 (-0.6m, 0.5m, 0.3m)

**자연어 입력:**
```
금속 판을 공구 선반으로 옮겨
```

**생성된 TDL:**
- Scene에서 `workpiece_metal_plate`와 `tool_rack` 자동 인식
- 실제 위치 좌표를 사용한 이동 경로 생성
- 거리 자동 계산하여 적절한 속도 설정

---

## 🏗️ 시스템 아키텍처

```
[자연어 입력]
    ↓
[MuJoCo Scene XML] → [SceneParser] → [Scene Data]
    ↓                                      ↓
[NL Object Extractor] ← "용접기", "로봇 A"
    ↓
[Object Matching] → "welding_tool_01", "robot_arm_A"
    ↓
[Scene Context Builder]
    ↓
[Enhanced Prompt with Scene Info]
    ↓
[Gemini LLM]
    ↓
[Accurate TDL Code]
```

---

## 🔧 핵심 컴포넌트

### 1. MuJoCoSceneParser

**파일**: `mujoco_scene_parser.py`

**주요 메서드:**
```python
parser = MuJoCoSceneParser(model_path="scene.xml")

# Scene 파싱
scene_data = parser.parse_scene()

# 특정 객체 정보
obj_info = parser.get_object_info("welding_tool_01")

# 객체 이름 매칭
matched = parser.match_object_name("용접기")  # → "welding_tool_01"

# 거리 계산
distance = parser.calculate_distance("robot_arm_A", "welding_tool_01")
```

### 2. SceneContextBuilder

**파일**: `scene_context_builder.py`

**주요 메서드:**
```python
builder = SceneContextBuilder(scene_parser)

# 자연어에서 객체 추출 및 매칭
context = builder.build_context("용접기를 로봇으로 들어올려")

# LLM용 프롬프트 포맷팅
formatted = builder.format_for_prompt(context)
```

### 3. NL2TDLConverter (통합)

**파일**: `nl2tdl_converter.py`

**사용법:**
```python
converter = NL2TDLConverter(api_key="...")

# Scene parser와 함께 사용
result = converter.convert(
    nl_command="용접기를 집어올려",
    scene_parser=scene_parser  # ← MuJoCo scene 정보 활용
)
```

---

## 📊 Scene 데이터 구조

### Scene 파싱 결과

```python
{
    "bodies": {
        "welding_tool_01": {
            "id": 3,
            "position": [0.5, 0.3, 0.2],  # meters
            "rotation_euler": [0.0, 0.0, 0.0],  # degrees
            "mass": 15.0,  # kg
            "type": "body"
        },
        "robot_arm_A": {
            "id": 1,
            "position": [0.0, 0.0, 0.5],
            "mass": 10.0,
            "type": "body"
        }
    },
    "geoms": { ... },
    "joints": {
        "shoulder_pan": {
            "id": 0,
            "position": 0.0,  # radians (자동 degree 변환)
            "velocity": 0.0,
            "type": "hinge"
        }
    },
    "metadata": {
        "num_bodies": 10,
        "num_geoms": 15,
        "num_joints": 6,
        "gravity": [0, 0, -9.81]
    }
}
```

### LLM에 전달되는 컨텍스트

```
# SCENE INFORMATION (from MuJoCo Simulation)

The following objects are present in the current scene:

## Mentioned Objects

### **welding_tool_01** (referred as '용접기')
- Position: (0.500m, 0.300m, 0.200m)
- Rotation (Euler): (0.0°, 0.0°, 0.0°)
- Mass: 15.00 kg
- Type: body

### **robot_arm_A** (referred as '로봇')
- Position: (0.000m, 0.000m, 0.500m)
- Mass: 10.00 kg
- Type: body

## Robot Joint Positions
- shoulder_pan: 0.0°
- shoulder_lift: 0.0°
- elbow: 90.0°
...

## IMPORTANT INSTRUCTIONS
- Use the ACTUAL positions, masses, and rotations from the scene data above
- Convert positions from meters to millimeters when needed (1m = 1000mm)
```

---

## 🧪 테스트

### 자동 테스트 실행

```bash
python test_mujoco_integration.py
```

**테스트 항목:**
1. ✅ Scene Parser 기본 기능
2. ✅ Scene Context Builder
3. ✅ NL2TDL 통합

### 수동 테스트

```python
from mujoco_scene_parser import MuJoCoSceneParser
from nl2tdl_converter import NL2TDLConverter

# Scene 로드
parser = MuJoCoSceneParser("example_mujoco_scene.xml")

# Converter 초기화
converter = NL2TDLConverter(api_key="your-key")

# Scene과 함께 변환
result = converter.convert(
    "용접기를 집어올려",
    scene_parser=parser
)

print(result["tdl_code"])
```

---

## 🎓 고급 사용법

### 1. 실행 중인 시뮬레이션과 연동

```python
import mujoco
from mujoco_scene_parser import create_scene_parser_from_running_sim

# MuJoCo 시뮬레이션 실행 중
model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# 매 step마다 scene parser 업데이트
parser = create_scene_parser_from_running_sim(model, data)

while True:
    mujoco.mj_step(model, data)
    parser.update_data(data)  # 최신 상태 반영

    # 사용자 명령 처리
    result = converter.convert(nl_command, scene_parser=parser)
```

### 2. Scene 스냅샷 저장

```python
parser = MuJoCoSceneParser("scene.xml")

# JSON으로 저장
parser.export_scene_snapshot("scene_snapshot.json")
```

### 3. 커스텀 객체 매칭 로직

```python
from scene_context_builder import NLObjectExtractor

class CustomExtractor(NLObjectExtractor):
    def extract_objects(self, nl_command):
        # 커스텀 로직
        objects = super().extract_objects(nl_command)
        # 추가 처리
        return objects
```

---

## ⚠️ 주의사항 및 제한사항

### 현재 제한사항

1. **정적 Scene만 지원**: 실시간 동작 중인 시뮬레이션은 별도 업데이트 필요
2. **객체 타입 제한**: body, geom, joint만 파싱 (sensor, actuator는 미지원)
3. **좌표계**: 기본 MuJoCo 좌표계 사용 (변환 필요시 수동 처리)

### 권장 사항

1. **명확한 객체 이름**: XML에서 의미있는 이름 사용
   ```xml
   <!-- Good -->
   <body name="welding_tool_01" ...>

   <!-- Bad -->
   <body name="body1" ...>
   ```

2. **질량 정보 포함**: 정확한 로봇 제어를 위해 질량 설정
   ```xml
   <geom ... mass="15.0"/>
   ```

3. **적절한 임계값**: 객체 매칭 임계값 조정 가능
   ```python
   matched = parser.match_object_name("용접기", threshold=0.7)
   ```

---

## 🐛 문제 해결

### Q: MuJoCo 설치 오류
```
ImportError: No module named 'mujoco'
```

**A:** MuJoCo 설치
```bash
pip install mujoco
```

### Q: Scene 파일 로드 실패
```
FileNotFoundError: MuJoCo model file not found
```

**A:** 절대 경로 또는 상대 경로 확인
```python
import os
full_path = os.path.abspath("example_mujoco_scene.xml")
parser = MuJoCoSceneParser(full_path)
```

### Q: 객체가 매칭되지 않음
```
No match found for '용접기'
```

**A:**
1. Scene에 해당 객체가 있는지 확인
2. 객체 이름 확인: `parser.get_all_objects()`
3. 매칭 임계값 낮추기: `match_object_name("용접기", threshold=0.5)`

### Q: 생성된 TDL에 Scene 정보가 반영 안됨

**A:**
1. `scene_parser` 파라미터 전달 확인
2. Scene 정보가 프롬프트에 포함되는지 로그 확인
3. LLM이 Scene 정보를 무시할 수도 있음 (재시도)

---

## 📚 추가 자료

### Example Scene Files

- `example_mujoco_scene.xml`: 용접 작업장 시나리오
- 로봇 팔, 용접 공구, 작업물 포함
- 다양한 객체 타입 예시

### API Reference

자세한 API 문서는 각 모듈의 docstring 참조:
- `mujoco_scene_parser.py`
- `scene_context_builder.py`
- `nl2tdl_converter.py`

---

## 🔮 향후 계획

- [ ] 실시간 시뮬레이션 상태 추적
- [ ] 센서/액추에이터 정보 통합
- [ ] 충돌 감지 및 경로 계획
- [ ] 다중 로봇 협업 시나리오
- [ ] ROS/Gazebo 연동

---

## 📞 지원

- 버그 리포트: GitHub Issues
- 질문: README.md 참조
- 예제: `test_mujoco_integration.py` 참조

---

**Version**: 1.0.0 (MuJoCo Integration)
**Date**: 2025-11-12
**Powered by**: MuJoCo + Google Gemini AI
