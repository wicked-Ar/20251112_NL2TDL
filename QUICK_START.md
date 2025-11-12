# NL2TDL Converter - 빠른 시작 가이드

## 🚀 5분 만에 시작하기

### 1단계: 설치 (1분)

```bash
# 프로젝트 디렉토리로 이동
cd NL2TDL

# 필수 라이브러리 설치
pip install -r requirements.txt
```

**선택사항 - MuJoCo 기능 사용 시:**
```bash
pip install mujoco numpy
```

---

### 2단계: 실행 (1분)

```bash
python main.py
```

---

### 3단계: 모드 선택 (1분)

프로그램이 실행되면 다음 메뉴가 나타납니다:

```
======================================================================
  실행 모드 선택
======================================================================

1. 대화형 모드 (Interactive Mode)
   - 명령을 하나씩 입력하며 변환합니다.

2. 배치 모드 (Batch Mode)
   - 텍스트 파일에서 여러 명령을 한번에 처리합니다.

3. 단일 명령 모드 (Single Command Mode)
   - 한 개의 명령만 빠르게 변환합니다.

4. MuJoCo Scene 연동 모드 (MuJoCo Mode) ⭐NEW
   - MuJoCo 시뮬레이션 scene 정보를 활용하여 변환합니다.

0. 종료 (Exit)

선택 (0-4):
```

---

## 💡 사용 예시

### 예시 1: 대화형 모드 (가장 간단!)

```
선택 (0-4): 1

자연어 명령> 로봇을 홈 위치로 이동시켜

🔄 변환 중...
✓ TDL 코드 생성 성공!

[생성된 TDL 코드가 출력됩니다]

파일로 저장하시겠습니까? (y/n): y
✓ 저장 완료: output/nl2tdl_20251112_150000_로봇을_홈_위치로.tdl
```

### 예시 2: 단일 명령 모드

```
선택 (0-4): 3

자연어 명령을 입력하세요: 디지털 출력 포트 1번을 켜줘

🔄 변환 중...
✓ TDL 코드 생성 성공!

GOAL Execute_Process()
{
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
}
```

### 예시 3: 배치 모드 (여러 명령 한번에)

```
선택 (0-4): 2

입력 파일 경로: examples/example_commands.txt

📋 15개의 명령을 처리합니다...

[1/15] 처리 중: 로봇을 홈 위치로 이동시켜
  ✓ 변환 성공 → nl2tdl_20251112_150001_로봇을.tdl

[2/15] 처리 중: 디지털 출력 포트 1번을 켜줘
  ✓ 변환 성공 → nl2tdl_20251112_150002_디지털.tdl

...

배치 처리 완료: 15/15 성공
```

### 예시 4: MuJoCo Scene 연동 모드 ⭐

```
선택 (0-4): 4

MuJoCo XML 파일 경로: examples/example_mujoco_scene.xml

🔧 MuJoCo scene 로드 중...
✓ Scene 로드 완료

📊 Scene 정보:
   - Bodies: 10
   - Geoms: 15
   - Joints: 6

🎯 감지된 객체:
   1. robot_arm_A (body)
   2. welding_tool_01 (body)
   3. workpiece_metal_plate (body)
   ...

자연어 명령 (with scene)> 무게 15kg인 용접기를 로봇으로 집어올려

🔄 변환 중 (scene 정보 포함)...
✓ TDL 코드 생성 성공!

[Scene의 실제 위치(0.5m, 0.3m, 0.2m)와 질량(15kg)이 반영된 TDL 코드]

GOAL Initialize_Process()
{
    SPAWN SetWorkpieceWeight(15.0, PosX(0, 0, 0, 0, 0, 0)) WITH WAIT;
}

GOAL Execute_Process()
{
    SPAWN MoveLinear(PosX(500, 300, 200, 180, 0, 180), 60, 60, 0, 0) WITH WAIT;
}
```

---

## 📁 프로젝트 구조

```
NL2TDL/
├── main.py                    # ⭐ 메인 실행 파일
├── config.json                # 설정 (API 키)
├── requirements.txt           # 필수 라이브러리
│
├── core/                      # 핵심 모듈
│   ├── nl2tdl_converter.py   # TDL 변환 엔진
│   ├── mujoco_scene_parser.py # MuJoCo scene 파서
│   └── scene_context_builder.py # Scene 컨텍스트 빌더
│
├── examples/                  # 예제 파일
│   ├── example_commands.txt  # 예제 자연어 명령
│   └── example_mujoco_scene.xml # 예제 MuJoCo scene
│
├── docs/                      # 문서
│   ├── README.md             # 프로젝트 소개
│   ├── USAGE_GUIDE.md        # 상세 사용법
│   └── MUJOCO_INTEGRATION_GUIDE.md # MuJoCo 가이드
│
├── tests/                     # 테스트
│   ├── test_example.py       # 기본 테스트
│   └── test_mujoco_integration.py # MuJoCo 테스트
│
└── output/                    # 생성된 TDL 파일 (자동 생성)
```

---

## 🎓 다음 단계

### 기본 사용법 익히기
1. **대화형 모드**로 몇 가지 명령 시도
2. **예제 파일** (`examples/example_commands.txt`) 참고
3. 생성된 TDL 코드 확인

### 고급 기능 탐구
1. **MuJoCo 연동** - 실제 시뮬레이션 scene 활용
2. **배치 처리** - 여러 명령 자동화
3. **프로그래밍 방식** - Python 코드에서 직접 사용

### 추가 학습 자료
- 📖 **상세 가이드**: `docs/USAGE_GUIDE.md`
- 🤖 **MuJoCo 가이드**: `docs/MUJOCO_INTEGRATION_GUIDE.md`
- 📋 **프로젝트 요약**: `docs/PROJECT_SUMMARY.md`

---

## ⚙️ 설정

### API 키 설정

**방법 1: config.json 파일 수정**
```json
{
  "gemini_api_key": "your-api-key-here",
  "model_name": "gemini-2.0-flash-exp"
}
```

**방법 2: 환경변수**
```bash
export GEMINI_API_KEY="your-api-key-here"
```

**API 키 발급**: https://aistudio.google.com/app/apikey

---

## 🐛 문제 해결

### Q: API 키 오류
```
⚠ [오류] Google Gemini API 키가 설정되지 않았습니다.
```
→ `config.json` 파일의 `gemini_api_key` 확인

### Q: MuJoCo 설치 오류
```
⚠ MuJoCo가 설치되지 않았습니다.
```
→ `pip install mujoco numpy`

### Q: 모듈을 찾을 수 없음
```
ModuleNotFoundError: No module named 'core'
```
→ NL2TDL 디렉토리에서 실행하는지 확인: `cd NL2TDL && python main.py`

---

## 💬 자주 하는 질문

**Q: 어떤 자연어 명령을 사용할 수 있나요?**
- 로봇 동작: "로봇을 ~로 이동", "~를 집어올려"
- I/O 제어: "출력 ~를 켜/꺼줘", "입력 대기"
- 시간 제어: "~초 대기", "정지"
- 예제: `examples/example_commands.txt` 참조

**Q: 생성된 TDL 파일은 어디에 저장되나요?**
- `output/` 폴더에 자동 저장됩니다
- 파일명: `nl2tdl_[timestamp]_[명령요약].tdl`

**Q: MuJoCo 없이 사용할 수 있나요?**
- 네! 모드 1, 2, 3은 MuJoCo 없이 사용 가능합니다
- 모드 4만 MuJoCo가 필요합니다

**Q: 프로그래밍 방식으로 사용하려면?**
```python
from core import NL2TDLConverter

converter = NL2TDLConverter(api_key="your-key")
result = converter.convert("로봇을 홈으로 이동")
print(result["tdl_code"])
```

---

## 📞 도움말

- **기본 사용법**: 이 문서 (QUICK_START.md)
- **상세 가이드**: docs/USAGE_GUIDE.md
- **MuJoCo 연동**: docs/MUJOCO_INTEGRATION_GUIDE.md
- **문제 보고**: GitHub Issues

---

**버전**: 1.0.0
**업데이트**: 2025-11-12
