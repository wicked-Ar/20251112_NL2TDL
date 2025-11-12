# NL2TDL Converter

**자연어를 TDL(Task Description Language) 코드로 자동 변환하는 AI 기반 시스템**

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-Optional-orange.svg)](https://mujoco.org/)

---

## ⚡ 빠른 시작

```bash
# 1. 라이브러리 설치
pip install -r requirements.txt

# 2. 실행
python main.py

# 또는 빠른 실행 스크립트
python run.py
```

**자세한 가이드**: [QUICK_START.md](QUICK_START.md) 참조

---

## 🎯 주요 기능

✅ **자연어 이해** - "로봇을 홈 위치로 이동" 같은 일반 언어 입력
✅ **자동 TDL 생성** - 완전한 TDL 코드 자동 생성
✅ **MuJoCo 연동** ⭐ - 시뮬레이션 scene 정보 활용
✅ **다양한 실행 모드** - 대화형, 배치, 단일 명령
✅ **문법 검증** - 생성된 코드 자동 검증

---

## 📖 사용 예시

### 입력 (자연어)
```
무게 15kg인 용접기를 로봇으로 집어올려
```

### 출력 (TDL 코드)
```tdl
GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}

GOAL Initialize_Process()
{
    SPAWN SetWorkpieceWeight(15.0, PosX(0, 0, 0, 0, 0, 0)) WITH WAIT;
}

GOAL Execute_Process()
{
    SPAWN MoveLinear(PosX(500, 300, 200, 180, 0, 180), 60, 60, 0, 0) WITH WAIT;
}

GOAL Finalize_Process()
{
    SPAWN End() WITH WAIT;
}
```

---

## 📁 프로젝트 구조

```
NL2TDL/
├── main.py                 ⭐ 메인 실행 파일
├── run.py                  ⭐ 빠른 실행 스크립트
├── QUICK_START.md          ⭐ 빠른 시작 가이드
│
├── core/                   # 핵심 모듈
│   ├── nl2tdl_converter.py
│   ├── mujoco_scene_parser.py
│   └── scene_context_builder.py
│
├── examples/               # 예제 파일
├── docs/                   # 상세 문서
├── tests/                  # 테스트
└── output/                 # 생성된 TDL 파일
```

---

## 🚀 실행 방법

### 방법 1: 메인 프로그램

```bash
python main.py
```

메뉴에서 원하는 모드 선택:
- **1. 대화형 모드** - 명령을 하나씩 입력
- **2. 배치 모드** - 파일에서 여러 명령 처리
- **3. 단일 명령 모드** - 한 개 명령만 빠르게 변환
- **4. MuJoCo 모드** ⭐ - Scene 정보 활용

### 방법 2: 빠른 실행 스크립트

```bash
# 메인 메뉴
python run.py

# 직접 변환
python run.py "로봇을 홈 위치로 이동"

# 테스트
python run.py --test

# 도움말
python run.py --help
```

### 방법 3: 프로그래밍 방식

```python
from core import NL2TDLConverter

converter = NL2TDLConverter(api_key="your-api-key")
result = converter.convert("로봇을 홈으로 이동")
print(result["tdl_code"])
```

---

## ⭐ MuJoCo 시뮬레이션 연동 (NEW!)

MuJoCo scene에서 객체 정보를 실시간으로 추출하여 더 정확한 TDL 생성:

```bash
python main.py
> 4  # MuJoCo 모드 선택
> examples/example_mujoco_scene.xml  # Scene 파일 로드
> 용접기를 로봇으로 집어올려  # 자연어 명령
```

**Scene의 실제 위치(0.5m, 0.3m, 0.2m)와 질량(15kg)이 자동으로 TDL에 반영됩니다!**

자세한 내용: [docs/MUJOCO_INTEGRATION_GUIDE.md](docs/MUJOCO_INTEGRATION_GUIDE.md)

---

## 📚 문서

- 📘 **[QUICK_START.md](QUICK_START.md)** - 5분 만에 시작하기
- 📗 **[docs/USAGE_GUIDE.md](docs/USAGE_GUIDE.md)** - 상세 사용 가이드
- 📙 **[docs/MUJOCO_INTEGRATION_GUIDE.md](docs/MUJOCO_INTEGRATION_GUIDE.md)** - MuJoCo 연동 가이드
- 📕 **[docs/PROJECT_SUMMARY.md](docs/PROJECT_SUMMARY.md)** - 프로젝트 요약

---

## ⚙️ 설정

### API 키 설정

**config.json** 파일 수정:
```json
{
  "gemini_api_key": "your-api-key-here",
  "model_name": "gemini-2.0-flash-exp"
}
```

API 키 발급: https://aistudio.google.com/app/apikey

---

## 🧪 테스트

```bash
# 모든 테스트 실행
python run.py --test

# 개별 테스트
python tests/test_example.py
python tests/test_mujoco_integration.py
```

---

## 📋 요구사항

### 필수
- Python 3.8+
- google-generativeai
- pytz

### 선택 (MuJoCo 연동 시)
- mujoco >= 3.0.0
- numpy >= 1.24.0

---

## 🎓 지원하는 자연어 명령

### 로봇 동작
- "로봇을 홈 위치로 이동"
- "관절 각도 (0, 0, 90, 0, 90, 0)으로 이동"
- "위치 (300, 200, 150)으로 직선 이동"

### I/O 제어
- "디지털 출력 포트 1번을 켜줘"
- "입력 포트 11번이 켜질 때까지 대기"

### 시간 제어
- "2초 대기"
- "5초간 정지"

### 복합 작업
- "속도 50%로 설정하고 위치 A로 이동"
- "출력 1을 켜고 3초 대기한 후 꺼줘"

더 많은 예제: [examples/example_commands.txt](examples/example_commands.txt)

---

## 🤝 기여

버그 리포트, 기능 제안, Pull Request 환영합니다!

---

## 📞 지원

- **빠른 시작**: QUICK_START.md
- **문제 해결**: docs/USAGE_GUIDE.md
- **이슈 보고**: GitHub Issues

---

## 📜 라이선스

MIT License

---

## 🙏 감사의 말

- **Google Gemini AI** - 자연어 이해 및 코드 생성
- **MuJoCo** - 로봇 시뮬레이션 (선택적 기능)

---

**Version**: 1.0.0
**Last Updated**: 2025-11-12
**Powered by**: Google Gemini AI + MuJoCo (Optional)

---

## 🚦 시작하기

1. ⚡ **[QUICK_START.md](QUICK_START.md)** 읽기 (5분)
2. 🏃 `python run.py` 실행
3. 💬 자연어 명령 입력
4. ✨ TDL 코드 생성!

**Happy Coding! 🤖**
