#!/usr/bin/env python3
# test_example.py
# NL2TDL Converter 간단한 테스트 스크립트

"""
이 스크립트는 NL2TDL Converter의 기본 기능을 테스트합니다.
API 키가 설정되어 있어야 합니다.
"""

from core.nl2tdl_converter import NL2TDLConverter, FileManager
import sys

def test_basic_conversion():
    """기본 변환 테스트"""
    print("="*70)
    print("NL2TDL Converter 테스트")
    print("="*70)

    # API 키 (실제 사용시 환경변수나 설정파일에서 읽기)
    API_KEY = "AIzaSyDC4H1qd3uBfkAPrSCxAkLYCN5LbOU8rk4"

    if not API_KEY or API_KEY == "YOUR_API_KEY_HERE":
        print("\n⚠ API 키가 설정되지 않았습니다.")
        return False

    try:
        # 변환기 초기화
        print("\n1. 변환기 초기화 중...")
        converter = NL2TDLConverter(api_key=API_KEY)
        print("   ✓ 초기화 완료")

        # 테스트 명령어들
        test_commands = [
            "로봇을 홈 위치로 이동시켜",
            "디지털 출력 포트 1번을 켜줘",
            "2초 대기",
        ]

        print(f"\n2. {len(test_commands)}개의 명령어 테스트 시작...")

        success_count = 0
        for i, cmd in enumerate(test_commands, 1):
            print(f"\n   [{i}] 테스트: '{cmd}'")
            print("   " + "-"*60)

            result = converter.convert(cmd, add_command_defs=False)

            if result["success"]:
                print("   ✓ 변환 성공")

                # TDL 코드 출력 (처음 5줄만)
                lines = result["tdl_code"].split('\n')[:5]
                for line in lines:
                    print(f"   {line}")
                print("   ...")

                success_count += 1
            else:
                print("   ✗ 변환 실패")
                for error in result["errors"]:
                    print(f"     - {error}")

        # 결과 요약
        print("\n" + "="*70)
        print(f"테스트 결과: {success_count}/{len(test_commands)} 성공")
        print("="*70)

        return success_count == len(test_commands)

    except Exception as e:
        print(f"\n⚠ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_with_file_save():
    """파일 저장 포함 테스트"""
    print("\n\n")
    print("="*70)
    print("파일 저장 기능 테스트")
    print("="*70)

    API_KEY = "AIzaSyDC4H1qd3uBfkAPrSCxAkLYCN5LbOU8rk4"

    try:
        converter = NL2TDLConverter(api_key=API_KEY)

        cmd = "로봇을 관절 각도 (0, 0, 90, 0, 90, 0)으로 이동"
        print(f"\n명령: '{cmd}'")

        result = converter.convert(cmd, add_command_defs=True)

        if result["success"]:
            print("✓ 변환 성공")

            # 파일 저장
            filepath = FileManager.save_tdl(
                result["tdl_code"],
                cmd,
                output_dir="test_output"
            )

            if filepath:
                print(f"✓ 파일 저장: {filepath}")
                return True
            else:
                print("✗ 파일 저장 실패")
                return False
        else:
            print("✗ 변환 실패")
            return False

    except Exception as e:
        print(f"⚠ 오류: {e}")
        return False

def main():
    """메인 테스트 실행"""
    print("\n")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║                                                            ║")
    print("║              NL2TDL Converter Test Suite                   ║")
    print("║                                                            ║")
    print("╚════════════════════════════════════════════════════════════╝")

    # 테스트 실행
    test1_passed = test_basic_conversion()
    test2_passed = test_with_file_save()

    # 최종 결과
    print("\n\n")
    print("="*70)
    print("최종 테스트 결과")
    print("="*70)
    print(f"  기본 변환 테스트: {'✓ PASS' if test1_passed else '✗ FAIL'}")
    print(f"  파일 저장 테스트: {'✓ PASS' if test2_passed else '✗ FAIL'}")
    print("="*70)

    all_passed = test1_passed and test2_passed
    if all_passed:
        print("\n✓ 모든 테스트 통과!")
    else:
        print("\n⚠ 일부 테스트 실패")

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
