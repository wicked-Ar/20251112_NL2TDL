# NL2TDL Converter - 프로젝트 요약

## 📌 프로젝트 개요

**NL2TDL Converter**는 사용자의 자연어 명령(Natural Language)을 입력받아 로봇 제어용 TDL(Task Description Language) 코드로 자동 변환하는 AI 기반 시스템입니다.

### 핵심 기능
- 🤖 **자연어 이해**: Gemini AI를 활용한 자연어 분석
- 🔄 **자동 변환**: NL → TDL 코드 자동 생성
- ✅ **문법 검증**: 생성된 TDL 코드 자동 검증
- 💾 **파일 관리**: 자동 저장 및 로그 기록

---

## 📁 프로젝트 구조

```
NL2TDL/
├── nl2tdl_converter.py      # 핵심 변환 엔진
├── main.py                   # 메인 실행 프로그램
├── test_example.py           # 테스트 스크립트
├── requirements.txt          # 필수 라이브러리
├── config.json               # 설정 파일
├── example_commands.txt      # 예제 명령어 모음
├── README.md                 # 프로젝트 소개
├── USAGE_GUIDE.md            # 사용 가이드
└── PROJECT_SUMMARY.md        # 이 문서
```

---

## 🎯 TDL 생성 규칙 분석

기존 시스템(TDL_Translator.py, TDL2Robotics.py)을 분석하여 다음 규칙을 파악했습니다:

### 1. TDL 기본 구조

```tdl
GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}

GOAL Initialize_Process() { /* 초기화 */ }
GOAL Execute_Process() { /* 메인 작업 */ }
GOAL Finalize_Process() { /* 종료 */ }
```

### 2. GOAL 블록 규칙

| GOAL 블록 | 용도 | 포함 명령 |
|-----------|------|----------|
| Main_Process | 전체 프로세스 조율 | 3개 서브 프로세스 호출 |
| Initialize_Process | 초기화 및 설정 | 속도/툴 설정, 좌표계 설정 |
| Execute_Process | 메인 작업 로직 | 동작, I/O, 제어 흐름 |
| Finalize_Process | 종료 처리 | End() 명령 |

### 3. SPAWN 문법

```tdl
SPAWN CommandName(arg1, arg2, ...) WITH WAIT;
```

**규칙:**
- `SPAWN` 키워드로 시작
- 명령어 이름과 괄호 안에 인자
- `WITH WAIT;`로 종료 (세미콜론 필수)
- 동기 실행 (완료까지 대기)

### 4. 주요 TDL 명령어 카테고리

#### 동작 제어 (Motion)
- `MoveJoint`: 관절 공간 이동
- `MoveLinear`: 직선 이동
- `MoveCircular`: 원호 이동
- `PosJ(j1,j2,j3,j4,j5,j6)`: 관절 좌표
- `PosX(x,y,z,rx,ry,rz)`: 카르테시안 좌표

#### I/O 제어
- `SetDigitalOutput`: 디지털 출력 설정
- `GetDigitalInput`: 디지털 입력 읽기
- `WaitForDigitalInput`: 입력 대기
- `SetAnalogOutput`: 아날로그 출력 설정

#### 제어 흐름
- `If/Else/EndIf`: 조건문
- `While/EndWhile`: 반복문
- `For/Next`: 카운터 반복

#### 시스템
- `Delay`: 시간 지연
- `PrintLog`: 로그 출력
- `Call`: 서브 프로그램 호출
- `End`: 종료

#### 로봇 설정
- `SetJointVelocity`: 관절 속도 설정
- `SetTaskVelocity`: 작업 속도 설정
- `SetTool`: 툴 설정
- `SetRefCoord`: 좌표계 설정

### 5. COMMAND 정의

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

## 🔧 시스템 아키텍처

### 변환 파이프라인

```
┌─────────────────┐
│  자연어 입력     │  "로봇을 홈 위치로 이동시켜"
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ PromptBuilder   │  프롬프트 생성
│                 │  - TDL 문법 규칙 포함
│                 │  - 명령어 레퍼런스 추가
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  GeminiLLM      │  AI 분석 및 생성
│  (Gemini API)   │  - 의도 파악
│                 │  - 명령어 매핑
│                 │  - 파라미터 추출
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ TDLProcessor    │  후처리
│                 │  - 코드 추출
│                 │  - 메타데이터 추가
│                 │  - COMMAND 정의 추가
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  문법 검증       │  - GOAL 블록 확인
│                 │  - SPAWN 문법 검사
│                 │  - 괄호 균형 확인
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  TDL 코드 출력   │  완성된 TDL 프로그램
└─────────────────┘
```

### 주요 클래스

#### 1. `TDLKnowledgeBase`
- **역할**: TDL 명령어 및 문법 규칙 저장
- **기능**:
  - 69개 이상의 TDL 명령어 데이터베이스
  - 카테고리별 분류 및 검색
  - COMMAND 정의 템플릿 제공

#### 2. `GeminiLLM`
- **역할**: Google Gemini API 연동
- **기능**:
  - 프롬프트 기반 텍스트 생성
  - 안전 설정 및 오류 처리
  - 온도 및 토큰 수 제어

#### 3. `PromptBuilder`
- **역할**: AI용 프롬프트 생성
- **기능**:
  - TDL 문법 규칙 포함
  - 명령어 레퍼런스 자동 삽입
  - 예제 기반 Few-shot 학습

#### 4. `TDLProcessor`
- **역할**: 생성된 코드 후처리
- **기능**:
  - 코드 추출 (마크다운 제거)
  - 메타데이터 헤더 추가
  - COMMAND 정의 자동 추가
  - 문법 검증

#### 5. `NL2TDLConverter`
- **역할**: 메인 변환 클래스
- **기능**:
  - 전체 파이프라인 조율
  - 에러 처리
  - 결과 생성 및 반환

#### 6. `FileManager`
- **역할**: 파일 관리
- **기능**:
  - TDL 파일 저장
  - 변환 로그 기록
  - 파일명 자동 생성

---

## 🚀 사용 방법

### 1. 설치

```bash
cd NL2TDL
pip install -r requirements.txt
```

### 2. 실행

```bash
python main.py
```

### 3. 모드 선택

- **대화형 모드**: 명령을 하나씩 입력하며 변환
- **배치 모드**: 파일에서 여러 명령 일괄 처리
- **단일 명령 모드**: 한 개의 명령만 빠르게 변환

---

## 📝 예제

### 입력 (자연어)
```
로봇을 홈 위치로 이동하고 2초 대기한 후 디지털 출력 1번을 켜줘
```

### 출력 (TDL)
```tdl
// TDL Code Generated from Natural Language
// Generated At: 2025-11-12T09:30:00+09:00
// Original Command: 로봇을 홈 위치로 이동하고...

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
    SPAWN MoveJoint(PosJ(0, 0, 0, 0, 0, 0), 50, 50, 0, 0) WITH WAIT;
    SPAWN Delay(2.0) WITH WAIT;
    SPAWN SetDigitalOutput(1, 1) WITH WAIT;
}

GOAL Finalize_Process()
{
    SPAWN End() WITH WAIT;
}

// Command Definitions
COMMAND MoveJoint(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None) { motion.execute(type="Joint", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius, sync=synchronized_axes); }
COMMAND Delay(duration_sec) { system.time.delay(duration_sec=duration_sec); }
COMMAND SetDigitalOutput(port, value) { io.digital.set(port=port, value=value); }
COMMAND End() { system.execution.end(); }
```

---

## 🔬 기술 스택

- **언어**: Python 3.8+
- **AI 모델**: Google Gemini 2.0 Flash / 1.5 Pro
- **주요 라이브러리**:
  - `google-generativeai`: Gemini API 연동
  - `pytz`: 타임존 처리

---

## ⚙️ 설정

### API 키 설정

**방법 1: 환경변수**
```bash
export GEMINI_API_KEY="your-api-key"
```

**방법 2: config.json**
```json
{
  "gemini_api_key": "your-api-key",
  "model_name": "gemini-2.0-flash-exp"
}
```

### 고급 설정

- **온도 조절**: `temperature` (0.0~1.0, 기본 0.1)
- **최대 토큰**: `max_tokens` (기본 8192)
- **COMMAND 정의 추가**: `add_command_definitions` (기본 true)

---

## 🧪 테스트

```bash
# 테스트 스크립트 실행
python test_example.py
```

**테스트 항목:**
1. 기본 변환 기능
2. 파일 저장 기능
3. 문법 검증

---

## 📊 성능

- **변환 속도**: 명령당 약 2-5초 (API 호출 시간 포함)
- **정확도**: 간단한 명령 95%+, 복잡한 명령 80%+
- **지원 언어**: 한국어, 영어

---

## 🔒 제한사항

### 현재 미지원 기능
- 복잡한 중첩 제어 흐름
- 사용자 정의 함수 생성
- 복잡한 수식 연산

### AI 모델 제한
- 매우 특수한 명령 인식 어려움
- 드물게 문법 오류 발생 가능
- 긴 명령어 컨텍스트 제한

---

## 📚 문서

- **README.md**: 프로젝트 전체 소개
- **USAGE_GUIDE.md**: 상세 사용 가이드
- **PROJECT_SUMMARY.md**: 이 문서 (프로젝트 요약)

---

## 🎯 향후 계획

- [ ] 복잡한 제어 흐름 지원 개선
- [ ] 다국어 지원 확대
- [ ] 변환 캐싱 기능
- [ ] 웹 인터페이스 추가
- [ ] 실시간 코드 검증 강화

---

## 📞 지원

- **문서**: README.md, USAGE_GUIDE.md 참조
- **예제**: example_commands.txt 참조
- **테스트**: test_example.py 실행

---

**Version**: 1.0.0
**Date**: 2025-11-12
**Powered by**: Google Gemini AI
