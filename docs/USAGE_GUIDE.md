# NL2TDL Converter 사용 가이드

## 빠른 시작 (Quick Start)

### 1단계: 설치

```bash
cd NL2TDL
pip install -r requirements.txt
```

### 2단계: 실행

```bash
python main.py
```

### 3단계: 명령 입력

```
실행 모드 선택
==================
1. 대화형 모드
2. 배치 모드
3. 단일 명령 모드

선택 (1-3): 1

자연어 명령> 로봇을 홈 위치로 이동시켜
```

---

## TDL 생성 규칙 분석

기존 시스템에서 사용하는 TDL 생성 규칙을 분석한 결과입니다.

### 1. TDL 기본 구조

TDL은 다음과 같은 계층 구조를 가집니다:

```
1. Main_Process (메인 프로세스)
   ├── Initialize_Process (초기화)
   ├── Execute_Process (실행)
   └── Finalize_Process (종료)

2. COMMAND Definitions (명령어 정의)
```

### 2. GOAL 블록 규칙

#### Main_Process
- 항상 존재해야 함
- 세 개의 서브 프로세스를 순차적으로 호출

```tdl
GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}
```

#### Initialize_Process
- 로봇 설정 및 초기화 명령
- 속도 설정, 툴 설정, 좌표계 설정 등

```tdl
GOAL Initialize_Process()
{
    SPAWN SetJointVelocity(50) WITH WAIT;
    SPAWN SetTool(0) WITH WAIT;
}
```

#### Execute_Process
- 메인 작업 로직
- 동작, I/O, 제어 흐름 등

```tdl
GOAL Execute_Process()
{
    SPAWN MoveJoint(PosJ(0,0,90,0,90,0), 50, 50, 0, 0) WITH WAIT;
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
}
```

#### Finalize_Process
- 종료 처리
- 보통 End() 명령만 포함

```tdl
GOAL Finalize_Process()
{
    SPAWN End() WITH WAIT;
}
```

### 3. SPAWN 문법

모든 명령 실행은 SPAWN 문을 통해 이루어집니다:

```
SPAWN CommandName(arguments) WITH WAIT;
```

**규칙:**
- `SPAWN` 키워드로 시작
- 명령어 이름과 괄호 안에 인자
- `WITH WAIT;`로 종료 (세미콜론 필수)
- 동기 실행 (명령이 완료될 때까지 대기)

### 4. 명령어 카테고리별 규칙

#### 동작 명령 (Motion)

```tdl
SPAWN MoveJoint(target_pose, velocity, acceleration, tool, blending_radius) WITH WAIT;
SPAWN MoveLinear(target_pose, velocity, acceleration, tool, blending_radius) WITH WAIT;
```

**파라미터:**
- `target_pose`: PosJ() 또는 PosX() 함수
- `velocity`: 속도 (%, mm/s)
- `acceleration`: 가속도 (%, level)
- `tool`: 툴 ID (보통 0)
- `blending_radius`: 블렌딩 반경 (보통 0)

**포즈 정의:**
```tdl
PosJ(j1, j2, j3, j4, j5, j6)        # 관절 각도 (도)
PosX(x, y, z, rx, ry, rz)           # 카르테시안 좌표 (mm, 도)
```

#### I/O 명령

```tdl
SPAWN SetDigitalOutput(port, value) WITH WAIT;
SPAWN WaitForDigitalInput(port, value, timeout) WITH WAIT;
```

**파라미터:**
- `port`: 포트 번호 (1, 2, 3, ...)
- `value`: 0 (OFF) 또는 1 (ON)
- `timeout`: 타임아웃 시간 (초)

#### 제어 흐름

```tdl
SPAWN If(condition) WITH WAIT;
    SPAWN MoveJoint(...) WITH WAIT;
SPAWN Else() WITH WAIT;
    SPAWN MoveLinear(...) WITH WAIT;
SPAWN EndIf() WITH WAIT;
```

#### 시간 제어

```tdl
SPAWN Delay(duration_sec) WITH WAIT;
```

### 5. COMMAND 정의 규칙

각 명령어는 COMMAND 블록으로 정의됩니다:

```tdl
COMMAND CommandName(param1, param2, ...) {
    implementation;
}
```

**예시:**
```tdl
COMMAND Delay(duration_sec) {
    system.time.delay(duration_sec=duration_sec);
}

COMMAND MoveJoint(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None) {
    motion.execute(type="Joint", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius, sync=synchronized_axes);
}
```

---

## 자연어 → TDL 변환 프로세스

### 변환 파이프라인

```
1. 자연어 입력
   ↓
2. Gemini AI 분석
   - 의도 파악
   - 명령어 매핑
   - 파라미터 추출
   ↓
3. TDL 코드 생성
   - GOAL 블록 생성
   - SPAWN 문 작성
   ↓
4. 후처리
   - 메타데이터 헤더 추가
   - COMMAND 정의 추가
   ↓
5. 문법 검증
   - GOAL 블록 확인
   - SPAWN 문법 검사
   - 괄호 균형 확인
   ↓
6. 결과 반환
```

### 자연어 → TDL 매핑 예제

| 자연어 | TDL 명령어 | 비고 |
|--------|-----------|------|
| "이동", "움직여" | MoveJoint, MoveLinear | 컨텍스트에 따라 선택 |
| "대기", "기다려" | Delay | duration 파라미터 추출 |
| "켜줘", "ON" | SetDigitalOutput(port, 1) | 포트 번호 추출 |
| "꺼줘", "OFF" | SetDigitalOutput(port, 0) | 포트 번호 추출 |
| "용접 시작" | ArcOn | 용접 관련 키워드 인식 |
| "속도 설정" | SetJointVelocity, SetTaskVelocity | 단위에 따라 선택 |

### 파라미터 추론 규칙

#### 1. 위치 파라미터
```
자연어: "위치 (300, 200, 150)으로 이동"
추론: PosX(300, 200, 150, 180, 0, 180)
      - 회전 값은 기본값 (180, 0, 180) 사용
```

#### 2. 속도 파라미터
```
자연어: "속도 50%로"
추론: velocity=50 (관절 동작)
      또는 SetJointVelocity(50)

자연어: "속도 100mm/s로"
추론: velocity=100 (직선 동작)
      또는 SetTaskVelocity(100)
```

#### 3. 시간 파라미터
```
자연어: "2초 대기"
추론: Delay(2.0)

자연어: "10초 동안 대기"
추론: WaitForDigitalInput(..., timeout=10.0)
```

#### 4. 포트 번호
```
자연어: "출력 포트 1번"
추론: port=1

자연어: "디지털 입력 11"
추론: port=11
```

---

## 고급 사용법

### 1. 복잡한 명령 작성 팁

#### 순차 작업
```
자연어: "로봇을 홈으로 이동하고, 2초 대기한 후, 출력 1을 켜줘"

생성되는 TDL:
GOAL Execute_Process()
{
    SPAWN MoveJoint(PosJ(0,0,0,0,0,0), 50, 50, 0, 0) WITH WAIT;
    SPAWN Delay(2.0) WITH WAIT;
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
}
```

#### 조건부 작업
```
자연어: "입력 11이 켜지면 위치 A로 이동, 아니면 위치 B로 이동"

생성되는 TDL:
GOAL Execute_Process()
{
    SPAWN If(GetDigitalInput(11)==1) WITH WAIT;
        SPAWN MoveJoint(PosJ(...), 50, 50, 0, 0) WITH WAIT;
    SPAWN Else() WITH WAIT;
        SPAWN MoveJoint(PosJ(...), 50, 50, 0, 0) WITH WAIT;
    SPAWN EndIf() WITH WAIT;
}
```

### 2. 명령어 명확하게 작성하기

#### ❌ 모호한 표현
```
"로봇 움직여"              # 어디로? 어떻게?
"출력 켜줘"                # 몇 번 포트?
"빠르게 이동"              # 얼마나 빠르게?
```

#### ✅ 명확한 표현
```
"로봇을 관절 각도 (0,0,90,0,90,0)으로 이동"
"디지털 출력 포트 1번을 켜줘"
"속도 80%로 위치 (300,200,150)으로 이동"
```

### 3. 배치 처리 활용

대량의 명령을 한 번에 처리:

```bash
# commands.txt 파일 작성
로봇을 홈 위치로 이동
출력 1을 켜고 2초 대기
출력 1을 끄고 종료

# 배치 모드 실행
python main.py
> 2
> commands.txt
```

### 4. 프로그래밍 방식 통합

Python 스크립트에서 직접 사용:

```python
from nl2tdl_converter import NL2TDLConverter, FileManager

# 초기화
converter = NL2TDLConverter(api_key="your-api-key")

# 여러 명령 처리
commands = [
    "로봇을 홈으로 이동",
    "2초 대기",
    "출력 1 켜기"
]

for cmd in commands:
    result = converter.convert(cmd)
    if result["success"]:
        FileManager.save_tdl(result["tdl_code"], cmd)
```

---

## 문제 해결

### 일반적인 문제

#### 1. 생성된 코드에 원하는 명령이 없음

**원인:** 자연어 표현이 모호하거나 TDL 명령어와 매핑이 어려움

**해결:**
- 더 구체적으로 표현
- TDL 명령어 이름을 직접 언급
- 예: "로봇 이동" → "관절 공간으로 이동" 또는 "MoveJoint로 이동"

#### 2. 파라미터 값이 잘못됨

**원인:** AI가 파라미터를 잘못 추론

**해결:**
- 파라미터 값을 명시적으로 지정
- 예: "빠르게" → "속도 80%로"

#### 3. GOAL 블록에 명령이 잘못된 위치에 생성됨

**원인:** 명령의 목적이 불명확

**해결:**
- 명령의 의도를 명확히 표현
- 예: "속도 설정" (초기화) vs "이동" (실행)

### 디버깅 팁

#### 1. 로그 확인
```python
# conversion_log.json 파일 확인
# 변환 이력 및 오류 메시지 확인
```

#### 2. 단계별 테스트
```
복잡한 명령을 단순한 명령으로 나누어 테스트
```

#### 3. 예제 참조
```
example_commands.txt 파일의 예제 참조
유사한 명령 찾아서 변형
```

---

## 성능 최적화

### 1. 모델 선택

```python
# 빠른 응답이 필요한 경우
model = "gemini-2.0-flash-exp"  # 기본

# 더 정확한 결과가 필요한 경우
model = "gemini-1.5-pro"
```

### 2. 배치 처리

대량의 명령은 배치 모드 사용:
- 한 번에 여러 명령 처리
- 자동 저장으로 수작업 감소

### 3. 캐싱 (향후 기능)

자주 사용하는 명령은 캐싱하여 API 호출 감소

---

## 제한사항

### 1. 현재 지원하지 않는 기능

- 복잡한 중첩 제어 흐름 (다중 if-else, 중첩 반복문)
- 사용자 정의 함수 생성
- 변수 연산 (복잡한 수식)

### 2. AI 모델 제한

- 매우 특수한 로봇 명령은 인식 못할 수 있음
- 드물게 문법 오류 발생 가능
- 컨텍스트 길이 제한 (매우 긴 명령)

### 3. 언어 제한

- 현재 한국어와 영어 주로 지원
- 다른 언어는 테스트 필요

---

## 추가 리소스

### TDL 명령어 전체 목록

자세한 내용은 `nl2tdl_converter.py` 파일의 `TDLKnowledgeBase` 클래스 참조

### 예제 모음

`example_commands.txt` 파일에 다양한 예제 수록

### 기술 지원

- 버그 리포트: GitHub Issues
- 질문: README.md 참조

---

**Happy Coding! 🤖**
