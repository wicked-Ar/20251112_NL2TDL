# main.py
# NL2TDL Converter 메인 실행 프로그램
# 사용자 인터페이스를 제공하는 메인 애플리케이션

import os
import sys
import logging
from typing import Optional

from core.nl2tdl_converter import NL2TDLConverter, FileManager, print_banner

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==========================================================================
# 설정 관리
# ==========================================================================
class Config:
    """설정 관리 클래스"""

    DEFAULT_API_KEY = "AIzaSyDC4H1qd3uBfkAPrSCxAkLYCN5LbOU8rk4"
    DEFAULT_MODEL = "gemini-2.0-flash-exp"
    OUTPUT_DIR = "output"

    @classmethod
    def get_api_key(cls) -> str:
        """API 키 가져오기 (환경변수 또는 기본값)"""
        return os.getenv("GEMINI_API_KEY", cls.DEFAULT_API_KEY)

    @classmethod
    def get_model_name(cls) -> str:
        """모델 이름 가져오기"""
        return os.getenv("GEMINI_MODEL", cls.DEFAULT_MODEL)

# ==========================================================================
# 대화형 모드
# ==========================================================================
def interactive_mode(converter: NL2TDLConverter):
    """대화형 모드로 실행"""
    print("\n" + "="*70)
    print("  대화형 모드")
    print("="*70)
    print("\n자연어 명령을 입력하면 TDL 코드로 변환합니다.")
    print("종료하려면 'quit', 'exit', 또는 'q'를 입력하세요.\n")

    conversion_count = 0

    while True:
        try:
            # 사용자 입력
            print("-" * 70)
            nl_command = input("\n자연어 명령> ").strip()

            # 종료 체크
            if nl_command.lower() in ['quit', 'exit', 'q']:
                print(f"\n총 {conversion_count}개의 명령을 변환했습니다.")
                print("프로그램을 종료합니다.\n")
                break

            if not nl_command:
                print("⚠ 명령을 입력해주세요.")
                continue

            # 변환 실행
            print("\n🔄 변환 중...")
            result = converter.convert(nl_command, add_command_defs=True)

            # 결과 출력
            print("\n" + "="*70)
            if result["success"]:
                print("✓ TDL 코드 생성 성공!")
                print("="*70 + "\n")
                print(result["tdl_code"])
                print("\n" + "="*70)

                conversion_count += 1

                # 경고 출력
                if result["warnings"]:
                    print("\n⚠ 경고:")
                    for warning in result["warnings"]:
                        print(f"  - {warning}")

                if result["errors"]:
                    print("\n⚠ 문법 검증 오류:")
                    for error in result["errors"]:
                        print(f"  - {error}")

                # 파일 저장 여부 확인
                save_choice = input("\n파일로 저장하시겠습니까? (y/n): ").strip().lower()
                if save_choice == 'y':
                    filepath = FileManager.save_tdl(
                        result["tdl_code"],
                        nl_command,
                        Config.OUTPUT_DIR
                    )
                    if filepath:
                        print(f"✓ 저장 완료: {filepath}")

                    # 로그 저장
                    FileManager.save_conversion_log(nl_command, result["tdl_code"], result)

            else:
                print("✗ TDL 코드 생성 실패")
                print("="*70)
                print("\n오류:")
                for error in result["errors"]:
                    print(f"  - {error}")

        except KeyboardInterrupt:
            print(f"\n\n총 {conversion_count}개의 명령을 변환했습니다.")
            print("프로그램을 종료합니다.\n")
            break
        except Exception as e:
            logger.error(f"오류 발생: {e}", exc_info=True)
            print(f"\n⚠ 오류가 발생했습니다: {e}\n")

# ==========================================================================
# 배치 모드
# ==========================================================================
def batch_mode(converter: NL2TDLConverter, input_file: str):
    """배치 모드로 실행 (파일에서 명령 읽기)"""
    print("\n" + "="*70)
    print("  배치 모드")
    print("="*70)

    try:
        # 파일 읽기
        with open(input_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # 주석과 빈 줄 제외
        commands = []
        for line in lines:
            line = line.strip()
            if line and not line.startswith('#'):
                commands.append(line)

        if not commands:
            print(f"\n⚠ '{input_file}' 파일에 변환할 명령이 없습니다.")
            return

        print(f"\n📋 {len(commands)}개의 명령을 처리합니다...\n")

        # 각 명령 처리
        results = []
        success_count = 0

        for i, nl_command in enumerate(commands, 1):
            print(f"\n[{i}/{len(commands)}] 처리 중: {nl_command}")
            print("-" * 70)

            result = converter.convert(nl_command, add_command_defs=True)
            results.append(result)

            if result["success"]:
                # 자동 저장
                filepath = FileManager.save_tdl(
                    result["tdl_code"],
                    nl_command,
                    Config.OUTPUT_DIR
                )

                if filepath:
                    print(f"  ✓ 변환 성공 → {os.path.basename(filepath)}")
                    success_count += 1

                # 로그 저장
                FileManager.save_conversion_log(nl_command, result["tdl_code"], result)

            else:
                print(f"  ✗ 변환 실패")
                for error in result["errors"]:
                    print(f"    - {error}")

        # 요약
        print("\n" + "="*70)
        print(f"  배치 처리 완료: {success_count}/{len(commands)} 성공")
        print("="*70 + "\n")

    except FileNotFoundError:
        print(f"\n⚠ 파일을 찾을 수 없습니다: {input_file}")
    except Exception as e:
        logger.error(f"배치 처리 중 오류: {e}", exc_info=True)
        print(f"\n⚠ 오류가 발생했습니다: {e}")

# ==========================================================================
# 단일 명령 모드
# ==========================================================================
def single_command_mode(converter: NL2TDLConverter):
    """단일 명령 모드"""
    print("\n" + "="*70)
    print("  단일 명령 모드")
    print("="*70)

    nl_command = input("\n자연어 명령을 입력하세요: ").strip()

    if not nl_command:
        print("⚠ 명령이 입력되지 않았습니다.")
        return

    print("\n🔄 변환 중...")
    result = converter.convert(nl_command, add_command_defs=True)

    # 결과 출력
    print("\n" + "="*70)
    if result["success"]:
        print("✓ TDL 코드 생성 성공!")
        print("="*70 + "\n")
        print(result["tdl_code"])
        print("\n" + "="*70)

        # 경고 출력
        if result["warnings"]:
            print("\n⚠ 경고:")
            for warning in result["warnings"]:
                print(f"  - {warning}")

        if result["errors"]:
            print("\n⚠ 문법 검증 오류:")
            for error in result["errors"]:
                print(f"  - {error}")

        # 파일 저장
        save_choice = input("\n파일로 저장하시겠습니까? (y/n): ").strip().lower()
        if save_choice == 'y':
            filepath = FileManager.save_tdl(
                result["tdl_code"],
                nl_command,
                Config.OUTPUT_DIR
            )
            if filepath:
                print(f"\n✓ 저장 완료: {filepath}")

            # 로그 저장
            FileManager.save_conversion_log(nl_command, result["tdl_code"], result)

    else:
        print("✗ TDL 코드 생성 실패")
        print("="*70)
        print("\n오류:")
        for error in result["errors"]:
            print(f"  - {error}")

# ==========================================================================
# 메뉴 시스템
# ==========================================================================
def mujoco_mode(converter: NL2TDLConverter):
    """MuJoCo scene 연동 모드"""
    print("\n" + "="*70)
    print("  MuJoCo Scene 연동 모드")
    print("="*70)

    # MuJoCo 라이브러리 확인
    try:
        from core.mujoco_scene_parser import MuJoCoSceneParser
    except ImportError:
        print("\n⚠ MuJoCo가 설치되지 않았습니다.")
        print("설치 방법: pip install mujoco")
        input("\nEnter 키를 눌러 메뉴로 돌아가기...")
        return

    # Scene 파일 경로 입력
    model_path = input("\nMuJoCo XML 파일 경로: ").strip()

    if not model_path:
        print("⚠ 파일 경로가 입력되지 않았습니다.")
        return

    if not os.path.exists(model_path):
        print(f"⚠ 파일을 찾을 수 없습니다: {model_path}")
        return

    try:
        # Scene parser 초기화
        print(f"\n🔧 MuJoCo scene 로드 중...")
        scene_parser = MuJoCoSceneParser(model_path=model_path)
        print("✓ Scene 로드 완료")

        # Scene 요약 출력
        scene_data = scene_parser.parse_scene()
        print(f"\n📊 Scene 정보:")
        print(f"   - Bodies: {len(scene_data['bodies'])}")
        print(f"   - Geoms: {len(scene_data['geoms'])}")
        print(f"   - Joints: {len(scene_data['joints'])}")

        # 객체 리스트 출력
        all_objects = scene_parser.get_all_objects()
        if all_objects:
            print(f"\n🎯 감지된 객체 (처음 10개):")
            for i, obj in enumerate(all_objects[:10], 1):
                print(f"   {i}. {obj['name']} ({obj['category']})")
            if len(all_objects) > 10:
                print(f"   ... 그 외 {len(all_objects) - 10}개")

        # 대화형 루프
        print("\n" + "="*70)
        print("자연어 명령을 입력하세요. (종료: 'quit', 'exit', 'q')")
        print("Scene 정보가 자동으로 TDL 생성에 활용됩니다.")
        print("="*70)

        while True:
            nl_command = input("\n자연어 명령 (with scene)> ").strip()

            if nl_command.lower() in ['quit', 'exit', 'q']:
                break

            if not nl_command:
                print("⚠ 명령을 입력해주세요.")
                continue

            # Scene 정보와 함께 변환
            print("\n🔄 변환 중 (scene 정보 포함)...")
            result = converter.convert(
                nl_command,
                add_command_defs=True,
                scene_parser=scene_parser
            )

            # 결과 출력
            print("\n" + "="*70)
            if result["success"]:
                print("✓ TDL 코드 생성 성공!")
                print("="*70 + "\n")
                print(result["tdl_code"])
                print("\n" + "="*70)

                # 경고 출력
                if result["warnings"]:
                    print("\n⚠ 경고:")
                    for warning in result["warnings"]:
                        print(f"  - {warning}")

                if result["errors"]:
                    print("\n⚠ 문법 검증 오류:")
                    for error in result["errors"]:
                        print(f"  - {error}")

                # 파일 저장
                save_choice = input("\n파일로 저장하시겠습니까? (y/n): ").strip().lower()
                if save_choice == 'y':
                    filepath = FileManager.save_tdl(
                        result["tdl_code"],
                        nl_command,
                        Config.OUTPUT_DIR
                    )
                    if filepath:
                        print(f"✓ 저장 완료: {filepath}")

                    # 로그 저장
                    FileManager.save_conversion_log(nl_command, result["tdl_code"], result)
            else:
                print("✗ TDL 코드 생성 실패")
                print("="*70)
                print("\n오류:")
                for error in result["errors"]:
                    print(f"  - {error}")

    except Exception as e:
        logger.error(f"MuJoCo 모드 실행 중 오류: {e}", exc_info=True)
        print(f"\n⚠ 오류 발생: {e}")


def show_menu():
    """메인 메뉴 표시"""
    print("\n" + "="*70)
    print("  실행 모드 선택")
    print("="*70)
    print("\n1. 대화형 모드 (Interactive Mode)")
    print("   - 명령을 하나씩 입력하며 변환합니다.")
    print("\n2. 배치 모드 (Batch Mode)")
    print("   - 텍스트 파일에서 여러 명령을 한번에 처리합니다.")
    print("\n3. 단일 명령 모드 (Single Command Mode)")
    print("   - 한 개의 명령만 빠르게 변환합니다.")
    print("\n4. MuJoCo Scene 연동 모드 (MuJoCo Mode) ⭐NEW")
    print("   - MuJoCo 시뮬레이션 scene 정보를 활용하여 변환합니다.")
    print("\n0. 종료 (Exit)")
    print("\n" + "="*70)

# ==========================================================================
# 메인 함수
# ==========================================================================
def main():
    """메인 함수"""
    print_banner()

    # API 키 확인
    api_key = Config.get_api_key()
    model_name = Config.get_model_name()

    if not api_key or api_key == "YOUR_API_KEY_HERE":
        print("\n⚠ [오류] Google Gemini API 키가 설정되지 않았습니다.")
        print("\n다음 중 하나의 방법으로 API 키를 설정하세요:")
        print("  1. 환경변수 GEMINI_API_KEY 설정")
        print("  2. main.py 파일의 Config.DEFAULT_API_KEY 수정")
        print("\nAPI 키는 https://aistudio.google.com/app/apikey 에서 발급받을 수 있습니다.")
        input("\nEnter 키를 눌러 종료...")
        return

    try:
        # 변환기 초기화
        print(f"\n🔧 NL2TDL Converter 초기화 중...")
        print(f"   - 모델: {model_name}")
        converter = NL2TDLConverter(api_key=api_key, model_name=model_name)
        print("✓ 초기화 완료\n")

        # 메인 루프
        while True:
            show_menu()
            choice = input("선택 (0-4): ").strip()

            if choice == '1':
                interactive_mode(converter)
            elif choice == '2':
                input_file = input("\n입력 파일 경로: ").strip()
                if input_file:
                    batch_mode(converter, input_file)
                else:
                    print("⚠ 파일 경로가 입력되지 않았습니다.")
            elif choice == '3':
                single_command_mode(converter)
            elif choice == '4':
                mujoco_mode(converter)
            elif choice == '0':
                print("\n프로그램을 종료합니다.\n")
                break
            else:
                print("\n⚠ 잘못된 선택입니다. 0-4 사이의 숫자를 입력하세요.")

            # 계속할지 확인
            if choice in ['1', '2', '3', '4']:
                continue_choice = input("\n다른 작업을 하시겠습니까? (y/n): ").strip().lower()
                if continue_choice != 'y':
                    print("\n프로그램을 종료합니다.\n")
                    break

    except Exception as e:
        logger.error(f"프로그램 실행 중 오류: {e}", exc_info=True)
        print(f"\n⚠ [오류] {e}")
        input("\nEnter 키를 눌러 종료...")

if __name__ == "__main__":
    main()
