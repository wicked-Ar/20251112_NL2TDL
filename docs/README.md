# NL2TDL Converter

**Natural Language to TDL (Task Description Language) Converter**

사용자의 자연어 명령을 입력받아 로봇 제어용 TDL 코드로 자동 변환하는 프로그램입니다.

## 📋 목차

1. [개요](#개요)
2. [기능](#기능)
3. [TDL이란?](#tdl이란)
4. [설치 방법](#설치-방법)
5. [사용 방법](#사용-방법)
6. [예제](#예제)
7. [시스템 구조](#시스템-구조)
8. [API 키 설정](#api-키-설정)

---

## 개요

NL2TDL Converter는 Google Gemini AI를 활용하여 자연어 명령을 TDL(Task Description Language) 코드로 자동 변환하는 도구입니다.

기존 시스템에서 Job File ↔ TDL 변환을 수행하는 것과 달리, 이 프로그램은 **사람의 자연어 지시**를 직접 TDL로 변환합니다.

### 주요 특징
- ✅ **자연어 이해**: "로봇을 홈 위치로 이동시켜" 같은 일반 언어로 명령 가능
- ✅ **Gemini AI 활용**: Google의 최신 생성형 AI 모델 사용
- ✅ **완전한 TDL 생성**: GOAL 블록, SPAWN 문, COMMAND 정의까지 자동 생성
- ✅ **문법 검증**: 생성된 코드의 기본 문법 자동 검증
- ✅ **다양한 실행 모드**: 대화형, 배치, 단일 명령 모드 지원

---

## 기능

### 1. 자연어 → TDL 변환
사용자의 자연어 명령을 분석하여 TDL 코드로 변환합니다.

### 2. 3가지 실행 모드

#### 대화형 모드 (Interactive Mode)
- 명령을 하나씩 입력하며 대화형으로 변환
- 실시간 피드백 및 저장 옵션 제공

#### 배치 모드 (Batch Mode)
- 텍스트 파일에 여러 명령을 작성하여 일괄 처리
- 대량의 명령을 자동으로 변환

#### 단일 명령 모드 (Single Command Mode)
- 한 개의 명령만 빠르게 변환
- 테스트 및 실험용

### 3. 자동 파일 관리
- 생성된 TDL 코드를 자동으로 파일로 저장
- 변환 이력을 JSON 로그로 기록

### 4. 문법 검증
- GOAL 블록 존재 확인
- SPAWN 문법 검증
- 괄호 균형 검사

---

## TDL이란?

**TDL (Task Description Language)** 는 로봇 작업을 표준화된 방식으로 정의하는 언어입니다.

### TDL 구조

```tdl
// 메인 프로세스
GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}

// 초기화
GOAL Initialize_Process()
{
    SPAWN SetJointVelocity(50) WITH WAIT;
}

// 메인 작업
GOAL Execute_Process()
{
    SPAWN MoveJoint(PosJ(0,0,90,0,90,0), 50, 50, 0, 0) WITH WAIT;
    SPAWN Delay(2.0) WITH WAIT;
}

// 종료
GOAL Finalize_Process()
{
    SPAWN End() WITH WAIT;
}
```

### 주요 TDL 명령어

#### 동작 제어
- `MoveJoint`: 관절 공간 이동
- `MoveLinear`: 직선 이동
- `MoveCircular`: 원호 이동

#### I/O 제어
- `SetDigitalOutput`: 디지털 출력 설정
- `WaitForDigitalInput`: 디지털 입력 대기
- `SetAnalogOutput`: 아날로그 출력 설정

#### 제어 흐름
- `If/Else/EndIf`: 조건문
- `While/EndWhile`: 반복문
- `For/Next`: 카운터 반복문

#### 시스템
- `Delay`: 대기
- `PrintLog`: 로그 출력
- `Assign`: 변수 할당

---

## 설치 방법

### 1. 필수 요구사항

- Python 3.8 이상
- Google Gemini API 키

### 2. 라이브러리 설치

```bash
pip install -r requirements.txt
```

필요한 라이브러리:
- `google-generativeai`: Gemini AI 연동
- `pytz`: 타임존 처리

### 3. API 키 설정

Google AI Studio에서 API 키 발급:
https://aistudio.google.com/app/apikey

---

## 사용 방법

### 기본 실행

```bash
python main.py
```

### 실행 모드 선택

프로그램 실행 후 메뉴에서 선택:

```
1. 대화형 모드 (Interactive Mode)
2. 배치 모드 (Batch Mode)
3. 단일 명령 모드 (Single Command Mode)
0. 종료 (Exit)
```

### 1. 대화형 모드 사용

```
자연어 명령> 로봇을 홈 위치로 이동시켜

🔄 변환 중...
✓ TDL 코드 생성 성공!

[생성된 TDL 코드 출력]

파일로 저장하시겠습니까? (y/n): y
✓ 저장 완료: output/nl2tdl_20251012_145030_로봇을_홈_위치로.tdl
```

### 2. 배치 모드 사용

1. 텍스트 파일 생성 (예: `commands.txt`)
```
로봇을 홈 위치로 이동
디지털 출력 포트 1을 켜줘
2초 대기
로봇을 원점으로 복귀
```

2. 배치 모드 실행
```bash
python main.py
> 2 (배치 모드 선택)
> commands.txt (파일 경로 입력)
```

### 3. 단일 명령 모드 사용

```bash
python main.py
> 3 (단일 명령 모드 선택)
> 로봇을 90도 회전시켜 (명령 입력)
```

---

## 예제

### 예제 1: 기본 동작

**입력:**
```
로봇을 홈 위치 (0,0,90,0,90,0)으로 이동시켜
```

**출력 (TDL):**
```tdl
GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}

GOAL Initialize_Process()
{
}

GOAL Execute_Process()
{
    SPAWN MoveJoint(PosJ(0, 0, 90, 0, 90, 0), 50, 50, 0, 0) WITH WAIT;
}

GOAL Finalize_Process()
{
    SPAWN End() WITH WAIT;
}
```

### 예제 2: I/O 제어

**입력:**
```
디지털 출력 포트 1번을 켜고 2초 대기한 후 다시 꺼줘
```

**출력 (TDL):**
```tdl
GOAL Execute_Process()
{
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
    SPAWN Delay(2.0) WITH WAIT;
    SPAWN SetDigitalOutput(1, 0) WITH WAIT;
}
```

### 예제 3: 복잡한 작업

**입력:**
```
로봇 속도를 50%로 설정하고, 위치 (300, 200, 150)으로 직선 이동한 후 용접을 수행해줘
```

**출력 (TDL):**
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

## 시스템 구조

### 파일 구조

```
NL2TDL/
├── nl2tdl_converter.py    # 핵심 변환 엔진
├── main.py                # 메인 실행 프로그램
├── requirements.txt       # 의존성 라이브러리
├── README.md              # 이 문서
├── config.json            # 설정 파일 (선택)
├── example_commands.txt   # 예제 명령어 모음
└── output/                # 생성된 TDL 파일 저장
    └── nl2tdl_*.tdl
```

### 주요 컴포넌트

#### 1. `TDLKnowledgeBase`
- TDL 명령어 및 문법 규칙 저장
- 69개 이상의 TDL 명령어 데이터베이스
- 카테고리별 분류 (motion, io, control_flow, etc.)

#### 2. `GeminiLLM`
- Google Gemini API 연동
- 프롬프트 기반 텍스트 생성
- 안전 설정 및 오류 처리

#### 3. `PromptBuilder`
- 자연어 → TDL 변환용 프롬프트 생성
- TDL 문법 규칙 및 예제 포함
- 컨텍스트 기반 프롬프트 최적화

#### 4. `TDLProcessor`
- 생성된 코드 후처리
- 문법 검증 및 오류 확인
- 메타데이터 헤더 추가
- COMMAND 정의 자동 추가

#### 5. `NL2TDLConverter`
- 메인 변환 클래스
- 전체 변환 파이프라인 조율
- 결과 생성 및 반환

#### 6. `FileManager`
- 파일 저장 및 관리
- 변환 로그 기록
- 자동 파일명 생성

---

## API 키 설정

### 방법 1: 환경변수 설정

#### Windows (PowerShell)
```powershell
$env:GEMINI_API_KEY="your-api-key-here"
```

#### Linux/Mac
```bash
export GEMINI_API_KEY="your-api-key-here"
```

### 방법 2: 코드 직접 수정

`main.py` 파일의 `Config` 클래스 수정:

```python
class Config:
    DEFAULT_API_KEY = "your-api-key-here"
```

### API 키 발급

1. https://aistudio.google.com/app/apikey 접속
2. Google 계정으로 로그인
3. "Create API Key" 클릭
4. 생성된 키 복사

---

## 고급 사용법

### 커스텀 모델 사용

환경변수로 다른 Gemini 모델 지정:

```bash
export GEMINI_MODEL="gemini-1.5-pro"
```

사용 가능한 모델:
- `gemini-2.0-flash-exp` (기본, 빠름)
- `gemini-1.5-pro` (더 정확함)
- `gemini-1.5-flash` (가장 빠름)

### 프로그래밍 방식 사용

```python
from nl2tdl_converter import NL2TDLConverter

# 변환기 초기화
converter = NL2TDLConverter(api_key="your-api-key")

# 변환 실행
result = converter.convert("로봇을 홈 위치로 이동")

# 결과 확인
if result["success"]:
    print(result["tdl_code"])
else:
    print("오류:", result["errors"])
```

---

## 문제 해결

### Q: API 키 오류가 발생합니다.
A: API 키가 올바르게 설정되었는지 확인하세요. 환경변수 또는 코드 내 설정을 확인하세요.

### Q: 생성된 코드에 문법 오류가 있습니다.
A: 일부 복잡한 명령은 오류가 발생할 수 있습니다. 명령을 더 구체적으로 작성하거나 단순하게 나누어 시도하세요.

### Q: 특정 TDL 명령어가 생성되지 않습니다.
A: 자연어 명령에 해당 기능을 명확히 언급해주세요. 예: "용접" 대신 "아크 용접 시작"

### Q: 변환 속도가 느립니다.
A: Gemini API 호출에 시간이 걸립니다. 배치 모드를 사용하거나 더 빠른 모델(`gemini-2.0-flash-exp`)을 사용하세요.

---

## 라이선스

이 프로젝트는 교육 및 연구 목적으로 제공됩니다.

---

## 기여

버그 리포트, 기능 제안, 또는 개선 사항이 있으시면 이슈를 등록해주세요.

---

## 변경 이력

### v1.0.0 (2025-01-12)
- 초기 릴리스
- Gemini AI 기반 NL → TDL 변환 기능
- 대화형, 배치, 단일 명령 모드 지원
- 자동 문법 검증 및 파일 저장

---

**Powered by Google Gemini AI**
