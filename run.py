#!/usr/bin/env python3
# run.py
# NL2TDL Converter 빠른 실행 스크립트

"""
NL2TDL Converter Quick Start Script

간단한 명령 하나로 NL2TDL Converter를 실행합니다.

사용법:
    python run.py                    # 메인 메뉴 실행
    python run.py "자연어 명령"      # 직접 변환
    python run.py --test             # 테스트 실행
    python run.py --help             # 도움말
"""

import sys
import os

def print_welcome():
    """환영 메시지"""
    print("""
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║              NL2TDL Converter - Quick Start                      ║
║         자연어 → TDL 코드 자동 생성기                             ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
""")

def show_help():
    """도움말 표시"""
    print("""
사용법:

1. 메인 메뉴 실행:
   python run.py

2. 직접 변환:
   python run.py "로봇을 홈 위치로 이동시켜"

3. 테스트:
   python run.py --test

4. 도움말:
   python run.py --help

빠른 시작 가이드: QUICK_START.md
상세 문서: docs/USAGE_GUIDE.md
""")

def quick_convert(nl_command: str):
    """빠른 변환 모드"""
    try:
        from core import NL2TDLConverter, FileManager
    except ImportError:
        print("\n⚠ 오류: core 모듈을 찾을 수 없습니다.")
        print("NL2TDL 디렉토리에서 실행하세요: cd NL2TDL && python run.py")
        return

    # API 키 확인
    try:
        import json
        with open('config.json', 'r', encoding='utf-8') as f:
            config = json.load(f)
            api_key = config.get('gemini_api_key', '')
    except:
        api_key = os.getenv('GEMINI_API_KEY', '')

    if not api_key:
        print("\n⚠ API 키가 설정되지 않았습니다.")
        print("config.json 파일에서 gemini_api_key를 설정하거나")
        print("환경변수 GEMINI_API_KEY를 설정하세요.")
        return

    print(f"\n🔄 변환 중: '{nl_command}'")
    print("-" * 70)

    # 변환 실행
    converter = NL2TDLConverter(api_key=api_key)
    result = converter.convert(nl_command, add_command_defs=True)

    if result["success"]:
        print("\n✓ 변환 성공!")
        print("=" * 70)
        print(result["tdl_code"])
        print("=" * 70)

        # 자동 저장
        filepath = FileManager.save_tdl(result["tdl_code"], nl_command)
        if filepath:
            print(f"\n✓ 저장 완료: {filepath}")

        # 경고/오류
        if result["warnings"]:
            print("\n⚠ 경고:")
            for w in result["warnings"]:
                print(f"  - {w}")
        if result["errors"]:
            print("\n⚠ 오류:")
            for e in result["errors"]:
                print(f"  - {e}")
    else:
        print("\n✗ 변환 실패")
        for error in result["errors"]:
            print(f"  - {error}")

def run_tests():
    """테스트 실행"""
    print("\n🧪 테스트 실행 중...")
    print("=" * 70)

    import subprocess

    # 기본 테스트
    print("\n[1/2] 기본 테스트...")
    try:
        result = subprocess.run([sys.executable, "tests/test_example.py"],
                              capture_output=False, text=True)
        if result.returncode == 0:
            print("✓ 기본 테스트 통과")
        else:
            print("✗ 기본 테스트 실패")
    except Exception as e:
        print(f"✗ 테스트 실행 오류: {e}")

    # MuJoCo 테스트
    print("\n[2/2] MuJoCo 통합 테스트...")
    try:
        result = subprocess.run([sys.executable, "tests/test_mujoco_integration.py"],
                              capture_output=False, text=True)
        if result.returncode == 0:
            print("✓ MuJoCo 테스트 통과")
        else:
            print("✗ MuJoCo 테스트 실패 (MuJoCo가 설치되지 않았을 수 있습니다)")
    except Exception as e:
        print(f"✗ 테스트 실행 오류: {e}")

    print("\n" + "=" * 70)

def main():
    """메인 함수"""
    print_welcome()

    # 인자 확인
    if len(sys.argv) > 1:
        arg = sys.argv[1]

        if arg in ['--help', '-h', 'help']:
            show_help()
            return

        elif arg in ['--test', '-t', 'test']:
            run_tests()
            return

        elif arg.startswith('-'):
            print(f"\n⚠ 알 수 없는 옵션: {arg}")
            show_help()
            return

        else:
            # 직접 변환 모드
            nl_command = ' '.join(sys.argv[1:])
            quick_convert(nl_command)
            return

    # 메인 프로그램 실행
    print("\n메인 프로그램을 시작합니다...")
    print("(종료: Ctrl+C)\n")

    try:
        import main
        main.main()
    except KeyboardInterrupt:
        print("\n\n프로그램을 종료합니다.")
    except ImportError:
        print("\n⚠ main.py를 찾을 수 없습니다.")
        print("NL2TDL 디렉토리에서 실행하세요.")
    except Exception as e:
        print(f"\n⚠ 오류: {e}")

if __name__ == "__main__":
    main()
