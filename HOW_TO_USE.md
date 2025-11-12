# 📖 NL2TDL Converter 사용 방법

## 목차
1. [프로그램 소개](#프로그램-소개)
2. [설치 방법](#설치-방법)
3. [사용 방법](#사용-방법)
4. [실전 예시](#실전-예시)
5. [문제 해결](#문제-해결)

---

## 프로그램 소개

### 이 프로그램이 하는 일

**NL2TDL Converter**는 일반 언어(자연어)를 로봇 제어 코드(TDL)로 자동 변환해주는 프로그램입니다.

**예시:**
- 당신이 말함: "로봇을 홈 위치로 이동시켜"
- 프로그램이 생성: 완전한 TDL 코드 (100줄 이상)

### 핵심 기능

1. **자연어 입력** - 프로그래밍 지식 없이도 사용 가능
2. **자동 코드 생성** - TDL 문법을 몰라도 OK
3. **MuJoCo 연동** - 시뮬레이션 정보를 활용한 정확한 코드 생성
4. **다양한 모드** - 상황에 맞는 사용 방법 선택

### 프로젝트 구조 이해하기

```
NL2TDL/
│
├── 📄 main.py              ⭐ 메인 실행 파일 (이것만 실행하면 됨!)
├── 📄 run.py               ⭐ 빠른 실행 스크립트
├── 📖 README.md            ⭐ 프로젝트 소개
├── 📖 QUICK_START.md       ⭐ 5분 시작 가이드
├── 📖 HOW_TO_USE.md        ⭐ 이 문서 (상세 사용법)
│
├── 📁 core/                # 핵심 프로그램 (건드릴 필요 없음)
│   ├── nl2tdl_converter.py
│   ├── mujoco_scene_parser.py
│   └── scene_context_builder.py
│
├── 📁 examples/            # 예제 파일들
│   ├── example_commands.txt        # 자연어 명령 예제
│   └── example_mujoco_scene.xml    # MuJoCo scene 예제
│
├── 📁 docs/                # 상세 문서들
│   ├── USAGE_GUIDE.md              # 상세 가이드
│   └── MUJOCO_INTEGRATION_GUIDE.md # MuJoCo 가이드
│
├── 📁 tests/               # 테스트 파일 (선택사항)
│
└── 📁 output/              # 생성된 TDL 파일 저장 (자동 생성)
```

---

## 설치 방법

### 1단계: Python 확인

이미 Python이 설치되어 있는지 확인:

```bash
python --version
```

Python 3.8 이상이어야 합니다.

### 2단계: 필수 라이브러리 설치

NL2TDL 폴더로 이동:

```bash
cd NL2TDL
```

라이브러리 설치:

```bash
pip install -r requirements.txt
```

**설치되는 것:**
- `google-generativeai` - Gemini AI (필수)
- `pytz` - 시간대 처리 (필수)
- `mujoco`, `numpy` - MuJoCo 연동 (선택, 모드 4 사용 시만)

### 3단계: API 키 설정

**config.json** 파일 열기:

```json
{
  "gemini_api_key": "여기에_API_키_입력",
  "model_name": "gemini-2.0-flash-exp"
}
```

**API 키 발급:**
1. https://aistudio.google.com/app/apikey 접속
2. Google 계정으로 로그인
3. "Create API Key" 클릭
4. 생성된 키 복사하여 config.json에 붙여넣기

---

## 사용 방법

### 기본 사용법 (가장 간단!)

#### 방법 1: 메인 프로그램 실행

```bash
python main.py
```

메뉴가 나타나면 원하는 번호 입력:

```
1. 대화형 모드        ← 추천! 가장 쉬움
2. 배치 모드          ← 여러 명령 한번에
3. 단일 명령 모드     ← 빠른 테스트
4. MuJoCo 모드        ← Scene 정보 활용
0. 종료
```

#### 방법 2: 빠른 실행 (고급 사용자)

```bash
# 메인 메뉴
python run.py

# 직접 변환
python run.py "로봇을 홈 위치로 이동"

# 테스트
python run.py --test
```

---

### 모드별 사용법

#### 모드 1: 대화형 모드 (Interactive Mode) ⭐ 추천

**언제 사용?**
- 처음 사용할 때
- 명령을 하나씩 시도하며 배우고 싶을 때
- 가장 편한 방법!

**사용 방법:**

1. 실행:
   ```bash
   python main.py
   ```

2. 메뉴에서 **1** 입력

3. 자연어 명령 입력:
   ```
   자연어 명령> 로봇을 홈 위치로 이동시켜
   ```

4. 결과 확인 후 저장 여부 선택:
   ```
   파일로 저장하시겠습니까? (y/n): y
   ```

5. 계속 다른 명령 입력 가능 (종료: `quit`)

**팁:**
- 명령은 편하게 한국어로 입력하세요
- 구체적일수록 좋은 결과가 나옵니다
- 예제 참고: `examples/example_commands.txt`

---

#### 모드 2: 배치 모드 (Batch Mode)

**언제 사용?**
- 여러 명령을 한번에 처리하고 싶을 때
- 반복 작업 자동화

**사용 방법:**

1. 텍스트 파일에 명령 작성 (`my_commands.txt`):
   ```
   로봇을 홈 위치로 이동
   2초 대기
   출력 포트 1번 켜기
   출력 포트 1번 끄기
   ```

2. 실행:
   ```bash
   python main.py
   ```

3. 메뉴에서 **2** 입력

4. 파일 경로 입력:
   ```
   입력 파일 경로: my_commands.txt
   ```

5. 자동으로 모든 명령 처리됨!

**팁:**
- 예제 파일 활용: `examples/example_commands.txt`
- `#`으로 시작하는 줄은 무시됨 (주석)

---

#### 모드 3: 단일 명령 모드 (Single Command)

**언제 사용?**
- 딱 하나의 명령만 빠르게 변환하고 싶을 때

**사용 방법:**

1. 실행:
   ```bash
   python main.py
   ```

2. 메뉴에서 **3** 입력

3. 명령 한 번만 입력하고 종료

**더 빠른 방법:**
```bash
python run.py "로봇을 홈으로 이동"
```

---

#### 모드 4: MuJoCo Scene 연동 모드 ⭐ 고급 기능

**언제 사용?**
- MuJoCo 시뮬레이션을 사용할 때
- Scene의 실제 객체 정보를 활용하고 싶을 때
- 더 정확한 TDL 코드가 필요할 때

**사용 방법:**

1. MuJoCo 설치 (처음 한 번만):
   ```bash
   pip install mujoco numpy
   ```

2. 실행:
   ```bash
   python main.py
   ```

3. 메뉴에서 **4** 입력

4. Scene 파일 경로 입력:
   ```
   MuJoCo XML 파일 경로: examples/example_mujoco_scene.xml
   ```

5. Scene 정보 확인 후 명령 입력:
   ```
   자연어 명령 (with scene)> 무게 15kg인 용접기를 집어올려
   ```

**무엇이 다른가?**
- Scene에서 객체의 **실제 위치** 자동 추출
- 객체의 **질량** 정보 활용
- "용접기" → `welding_tool_01` 자동 매칭

**결과 비교:**

일반 모드:
```tdl
SPAWN MoveLinear(PosX(300, 200, 150, ...), ...) WITH WAIT;
# ↑ 추정 위치
```

MuJoCo 모드:
```tdl
SPAWN SetWorkpieceWeight(15.0, ...) WITH WAIT;  # ← Scene의 실제 질량
SPAWN MoveLinear(PosX(500, 300, 200, ...), ...) WITH WAIT;
# ↑ Scene의 실제 위치 (0.5m → 500mm)
```

---

## 실전 예시

### 예시 1: 간단한 로봇 이동

**입력:**
```
로봇을 홈 위치로 이동시켜
```

**생성된 TDL (일부):**
```tdl
GOAL Execute_Process()
{
    SPAWN MoveJoint(PosJ(0, 0, 0, 0, 0, 0), 50, 50, 0, 0) WITH WAIT;
}
```

---

### 예시 2: I/O 제어

**입력:**
```
디지털 출력 포트 1번을 켜고 2초 대기한 후 다시 꺼줘
```

**생성된 TDL (일부):**
```tdl
GOAL Execute_Process()
{
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
    SPAWN Delay(2.0) WITH WAIT;
    SPAWN SetDigitalOutput(1, 0) WITH WAIT;
}
```

---

### 예시 3: 복잡한 작업

**입력:**
```
로봇 속도를 50%로 설정하고 위치 (300, 200, 150)으로 이동한 후 용접 시작
```

**생성된 TDL (일부):**
```tdl
GOAL Initialize_Process()
{
    SPAWN SetJointVelocity(50) WITH WAIT;
}

GOAL Execute_Process()
{
    SPAWN MoveLinear(PosX(300, 200, 150, 180, 0, 180), 60, 60, 0, 0) WITH WAIT;
    SPAWN ArcOn() WITH WAIT;
}
```

---

### 예시 4: MuJoCo Scene 활용

**Scene 정보:**
- `welding_tool_01`: 위치 (0.5m, 0.3m, 0.2m), 질량 15kg
- `robot_arm_A`: 관절 각도 (0°, 0°, 90°, 0°, 90°, 0°)

**입력:**
```
용접기를 로봇으로 집어올려
```

**생성된 TDL (Scene 정보 반영):**
```tdl
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

## 문제 해결

### Q1: API 키 오류

**증상:**
```
⚠ [오류] Google Gemini API 키가 설정되지 않았습니다.
```

**해결:**
1. `config.json` 파일 확인
2. `gemini_api_key` 값이 올바른지 확인
3. API 키에 따옴표 있는지 확인: `"your-key-here"`

---

### Q2: 모듈을 찾을 수 없음

**증상:**
```
ModuleNotFoundError: No module named 'core'
```

**해결:**
NL2TDL 폴더에서 실행하는지 확인:
```bash
cd NL2TDL
python main.py
```

---

### Q3: MuJoCo 설치 오류

**증상:**
```
⚠ MuJoCo가 설치되지 않았습니다.
```

**해결:**
```bash
pip install mujoco numpy
```

---

### Q4: 생성된 코드가 이상함

**해결 방법:**

1. **더 구체적인 명령 입력:**
   - ❌ "로봇 움직여"
   - ✅ "로봇을 관절 각도 (0, 0, 90, 0, 90, 0)으로 이동"

2. **예제 참고:**
   - `examples/example_commands.txt` 확인

3. **재시도:**
   - AI가 가끔 실수할 수 있음
   - 같은 명령을 다시 시도

---

### Q5: 파일이 저장되지 않음

**확인 사항:**
1. `output/` 폴더 확인
2. 권한 문제 - 관리자 권한으로 실행
3. 디스크 공간 확인

---

## 팁과 요령

### 좋은 자연어 명령 작성법

#### ✅ 좋은 예시
```
✓ "로봇을 관절 각도 (0, 0, 90, 0, 90, 0)으로 이동"
✓ "디지털 출력 포트 1번을 켜줘"
✓ "속도 50%로 위치 (300, 200, 150)으로 이동"
```

#### ❌ 나쁜 예시
```
✗ "로봇 움직여"           (어디로? 어떻게?)
✗ "출력 켜"               (몇 번 포트?)
✗ "빠르게"                (얼마나?)
```

### 효율적인 사용법

1. **처음엔 대화형 모드** - 익숙해질 때까지
2. **예제 활용** - `examples/example_commands.txt` 참고
3. **배치 모드로 자동화** - 반복 작업은 파일로
4. **MuJoCo 활용** - 정확한 코드 필요 시

---

## 다음 단계

### 초보자
1. ✅ 대화형 모드로 몇 가지 명령 시도
2. ✅ 예제 파일 확인
3. ✅ 생성된 TDL 코드 살펴보기

### 중급자
1. ✅ 배치 모드로 여러 명령 처리
2. ✅ 복잡한 명령 시도
3. ✅ 프로그래밍 방식 사용

### 고급자
1. ✅ MuJoCo Scene 연동
2. ✅ 커스텀 스크립트 작성
3. ✅ 프로젝트에 통합

---

## 추가 학습 자료

- 📘 **QUICK_START.md** - 5분 빠른 시작
- 📗 **docs/USAGE_GUIDE.md** - 상세 가이드
- 📙 **docs/MUJOCO_INTEGRATION_GUIDE.md** - MuJoCo 가이드
- 📕 **docs/PROJECT_SUMMARY.md** - 프로젝트 요약

---

## 요약

### 시작하기
```bash
cd NL2TDL
python main.py
1  # 대화형 모드 선택
로봇을 홈으로 이동  # 명령 입력
```

### 핵심 포인트
- ⭐ **가장 쉬운 방법**: `python main.py` → 모드 1
- ⭐ **여러 명령 처리**: 모드 2 (배치)
- ⭐ **정확한 코드**: 모드 4 (MuJoCo)

### 도움이 필요하면
- 이 문서 다시 읽기
- QUICK_START.md 확인
- 예제 파일 참고

---

**이제 시작하세요! 🚀**

```bash
python main.py
```
