#!/usr/bin/env python3
# test_mujoco_integration.py
# MuJoCo 시뮬레이션 연동 기능 테스트 스크립트

"""
MuJoCo Scene Parser 및 NL2TDL 변환기 통합 테스트

사용 방법:
1. MuJoCo 설치: pip install mujoco
2. 테스트 scene XML 파일 준비
3. 이 스크립트 실행: python test_mujoco_integration.py
"""

import sys
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==========================================================================
# Test 1: MuJoCo Scene Parser 기본 테스트
# ==========================================================================
def test_scene_parser():
    """Scene Parser 기본 기능 테스트"""
    print("\n" + "="*70)
    print("Test 1: MuJoCo Scene Parser 기본 테스트")
    print("="*70)

    try:
        from core.mujoco_scene_parser import MuJoCoSceneParser
        import mujoco
    except ImportError as e:
        print(f"\n⚠ Import 오류: {e}")
        print("MuJoCo 설치: pip install mujoco")
        return False

    # 간단한 테스트 scene 생성
    test_xml = """
    <mujoco>
        <worldbody>
            <body name="robot_arm" pos="0 0 0">
                <geom type="box" size="0.1 0.1 0.5" mass="10"/>
                <joint name="joint1" type="hinge" axis="0 0 1"/>
            </body>
            <body name="welding_tool" pos="0.5 0.3 0.2">
                <geom type="sphere" size="0.05" mass="15"/>
            </body>
            <body name="workpiece" pos="0.8 0 0.1">
                <geom type="box" size="0.2 0.1 0.05" mass="5"/>
            </body>
        </worldbody>
    </mujoco>
    """

    # 임시 파일 생성
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(test_xml)
        temp_path = f.name

    try:
        # Scene parser 생성
        print("\n1. Scene Parser 초기화...")
        parser = MuJoCoSceneParser(model_path=temp_path)
        print("   ✓ 초기화 성공")

        # Scene 파싱
        print("\n2. Scene 파싱...")
        scene_data = parser.parse_scene()
        print(f"   ✓ Bodies: {len(scene_data['bodies'])}")
        print(f"   ✓ Geoms: {len(scene_data['geoms'])}")
        print(f"   ✓ Joints: {len(scene_data['joints'])}")

        # 객체 정보 추출
        print("\n3. 객체 정보 추출...")
        all_objects = parser.get_all_objects()
        for obj in all_objects[:5]:  # 처음 5개만
            print(f"   - {obj['name']} ({obj['category']})")

        # 객체 이름 매칭 테스트
        print("\n4. 객체 이름 매칭 테스트...")
        test_queries = ["용접기", "로봇", "작업물"]
        for query in test_queries:
            matched = parser.match_object_name(query)
            print(f"   '{query}' → {matched}")

        print("\n✓ Test 1 통과!")
        return True

    except Exception as e:
        print(f"\n✗ Test 1 실패: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        import os
        os.unlink(temp_path)


# ==========================================================================
# Test 2: Scene Context Builder 테스트
# ==========================================================================
def test_context_builder():
    """Scene Context Builder 테스트"""
    print("\n" + "="*70)
    print("Test 2: Scene Context Builder 테스트")
    print("="*70)

    try:
        from core.mujoco_scene_parser import MuJoCoSceneParser
        from core.scene_context_builder import SceneContextBuilder, NLObjectExtractor
        import tempfile

        # 테스트 scene
        test_xml = """
        <mujoco>
            <worldbody>
                <body name="robot_A" pos="0 0 0">
                    <geom type="box" size="0.1 0.1 0.5" mass="20"/>
                    <joint name="shoulder" type="hinge"/>
                </body>
                <body name="welding_tool_01" pos="0.5 0.3 0.2">
                    <geom type="sphere" size="0.05" mass="15"/>
                </body>
            </worldbody>
        </mujoco>
        """

        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(test_xml)
            temp_path = f.name

        try:
            parser = MuJoCoSceneParser(model_path=temp_path)

            # 1. NL Object Extractor 테스트
            print("\n1. 자연어에서 객체 추출...")
            extractor = NLObjectExtractor()
            nl_command = "무게가 15kg인 용접기를 로봇 A로 들어올려"
            extracted = extractor.extract_objects(nl_command)
            print(f"   추출된 객체: {extracted}")

            # 2. Scene Context Builder 테스트
            print("\n2. Scene Context 생성...")
            builder = SceneContextBuilder(parser)
            context = builder.build_context(nl_command)
            print(f"   매칭된 객체 수: {len(context['scene_info']['mentioned_objects'])}")

            # 3. 프롬프트 포맷팅
            print("\n3. 프롬프트용 포맷팅...")
            formatted = builder.format_for_prompt(context)
            print("   생성된 프롬프트 섹션 (일부):")
            print("   " + "\n   ".join(formatted.split('\n')[:10]))

            print("\n✓ Test 2 통과!")
            return True

        finally:
            import os
            os.unlink(temp_path)

    except Exception as e:
        print(f"\n✗ Test 2 실패: {e}")
        import traceback
        traceback.print_exc()
        return False


# ==========================================================================
# Test 3: NL2TDL 변환기 통합 테스트
# ==========================================================================
def test_nl2tdl_integration():
    """NL2TDL 변환기와 MuJoCo Scene 통합 테스트"""
    print("\n" + "="*70)
    print("Test 3: NL2TDL 변환기 통합 테스트")
    print("="*70)

    try:
        from core.mujoco_scene_parser import MuJoCoSceneParser
        from core.nl2tdl_converter import NL2TDLConverter
        import tempfile

        # API 키 확인
        API_KEY = "AIzaSyDVz-UCGfPG54Zdinb02gLjSYenG8qMJsc"

        if not API_KEY or API_KEY == "YOUR_API_KEY_HERE":
            print("\n⚠ API 키가 설정되지 않았습니다.")
            return False

        # 테스트 scene
        test_xml = """
        <mujoco>
            <worldbody>
                <body name="robot_arm_A" pos="0 0 0">
                    <geom type="box" size="0.1 0.1 0.5" mass="20"/>
                    <joint name="joint1" type="hinge" axis="0 0 1"/>
                    <joint name="joint2" type="hinge" axis="0 1 0"/>
                </body>
                <body name="welding_tool" pos="0.5 0.3 0.2">
                    <geom type="sphere" size="0.05" mass="15"/>
                </body>
            </worldbody>
        </mujoco>
        """

        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(test_xml)
            temp_path = f.name

        try:
            # Scene parser 초기화
            print("\n1. Scene Parser 초기화...")
            scene_parser = MuJoCoSceneParser(model_path=temp_path)
            print("   ✓ Scene 로드 완료")

            # NL2TDL Converter 초기화
            print("\n2. NL2TDL Converter 초기화...")
            converter = NL2TDLConverter(api_key=API_KEY)
            print("   ✓ Converter 초기화 완료")

            # Scene 정보와 함께 변환
            print("\n3. Scene 정보를 활용한 TDL 생성...")
            nl_command = "무게 15kg인 용접기를 로봇 A로 집어올려"

            result = converter.convert(
                nl_command,
                add_command_defs=False,  # 빠른 테스트를 위해 COMMAND 정의는 생략
                scene_parser=scene_parser
            )

            # 결과 확인
            if result["success"]:
                print("   ✓ TDL 코드 생성 성공!")
                print("\n   생성된 TDL 코드 (처음 20줄):")
                lines = result["tdl_code"].split('\n')[:20]
                for line in lines:
                    print(f"   {line}")
                print("   ...")

                # Scene 정보가 포함되었는지 확인
                if "15" in result["tdl_code"] or "15.0" in result["tdl_code"]:
                    print("\n   ✓ Scene의 질량 정보(15kg)가 코드에 반영됨")
                if "0.5" in result["tdl_code"] or "500" in result["tdl_code"]:
                    print("   ✓ Scene의 위치 정보(0.5m)가 코드에 반영됨")

                print("\n✓ Test 3 통과!")
                return True
            else:
                print(f"\n✗ TDL 생성 실패: {result['errors']}")
                return False

        finally:
            import os
            os.unlink(temp_path)

    except Exception as e:
        print(f"\n✗ Test 3 실패: {e}")
        import traceback
        traceback.print_exc()
        return False


# ==========================================================================
# 메인 테스트 실행
# ==========================================================================
def main():
    """모든 테스트 실행"""
    print("\n")
    print("╔════════════════════════════════════════════════════════════════╗")
    print("║                                                                ║")
    print("║         MuJoCo Integration Test Suite for NL2TDL              ║")
    print("║                                                                ║")
    print("╚════════════════════════════════════════════════════════════════╝")

    tests = [
        ("Scene Parser 기본 기능", test_scene_parser),
        ("Scene Context Builder", test_context_builder),
        ("NL2TDL 통합", test_nl2tdl_integration),
    ]

    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"\n⚠ 테스트 '{test_name}' 실행 중 예외: {e}")
            results.append((test_name, False))

    # 최종 결과
    print("\n\n")
    print("="*70)
    print("최종 테스트 결과")
    print("="*70)

    for test_name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {test_name}: {status}")

    print("="*70)

    passed_count = sum(1 for _, passed in results if passed)
    total_count = len(results)

    if passed_count == total_count:
        print(f"\n✓ 모든 테스트 통과! ({passed_count}/{total_count})")
        return 0
    else:
        print(f"\n⚠ 일부 테스트 실패 ({passed_count}/{total_count} 통과)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
