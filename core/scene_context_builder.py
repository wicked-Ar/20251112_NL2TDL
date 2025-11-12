# scene_context_builder.py
# MuJoCo Scene 정보를 LLM이 이해할 수 있는 컨텍스트로 변환하는 모듈

import re
import logging
import numpy as np
from typing import Dict, List, Optional, Set, Tuple

logger = logging.getLogger(__name__)

# ==========================================================================
# 1. Natural Language Object Extractor
# ==========================================================================
class NLObjectExtractor:
    """
    자연어 명령에서 객체 참조를 추출하는 클래스

    예: "무게 15kg인 용접기를 로봇 A로 들어올려"
    → ["용접기", "로봇 A"]
    """

    # 일반적인 객체 키워드 패턴
    OBJECT_PATTERNS = [
        r'([가-힣a-zA-Z0-9_\s]+)\s*(?:를|을|이|가|에|의)',  # 조사가 붙은 명사
        r'로봇\s*[A-Z0-9가-힣]*',  # 로봇 이름
        r'[가-힣]+기',  # ~기로 끝나는 단어 (용접기, 집게 등)
        r'[가-힣]+봇',  # ~봇으로 끝나는 단어
        r'공구|툴|tool|gripper|arm',  # 일반 공구 키워드
    ]

    # 제외할 일반 단어
    EXCLUDE_WORDS = {
        '무게', '위치', '속도', '방향', '거리', '높이', '크기',
        '이동', '들어올려', '내려놔', '잡아', '놓아',
        '시작', '종료', '정지', '대기'
    }

    def __init__(self):
        self.patterns = [re.compile(p) for p in self.OBJECT_PATTERNS]

    def extract_objects(self, nl_command: str) -> List[str]:
        """
        자연어 명령에서 객체 참조 추출

        Args:
            nl_command: 자연어 명령

        Returns:
            추출된 객체 이름 리스트
        """
        objects = set()

        # 패턴 매칭으로 후보 추출
        for pattern in self.patterns:
            matches = pattern.findall(nl_command)
            for match in matches:
                # 조사 제거
                obj_name = self._clean_object_name(match)
                if obj_name and obj_name not in self.EXCLUDE_WORDS:
                    objects.add(obj_name)

        # 추가 휴리스틱: 숫자/문자가 포함된 명사구
        # 예: "로봇 A", "용접기 1"
        word_pattern = re.compile(r'([가-힣a-zA-Z]+\s*[A-Z0-9]+|[A-Z0-9]+\s*[가-힣a-zA-Z]+)')
        for match in word_pattern.findall(nl_command):
            obj_name = self._clean_object_name(match)
            if obj_name:
                objects.add(obj_name)

        logger.info(f"Extracted objects from NL: {list(objects)}")
        return list(objects)

    @staticmethod
    def _clean_object_name(name: str) -> str:
        """객체 이름 정제 (조사 제거 등)"""
        # 조사 제거
        name = re.sub(r'\s*(?:를|을|이|가|에|의|와|과|로|으로)\s*$', '', name)
        # 공백 정리
        name = ' '.join(name.split())
        return name.strip()


# ==========================================================================
# 2. Scene Context Builder
# ==========================================================================
class SceneContextBuilder:
    """
    MuJoCo Scene 데이터를 LLM용 컨텍스트로 변환하는 클래스

    주요 기능:
    - Scene 데이터 포맷팅
    - 자연어와 scene 객체 매칭
    - LLM용 프롬프트 섹션 생성
    """

    def __init__(self, scene_parser):
        """
        Scene Context Builder 초기화

        Args:
            scene_parser: MuJoCoSceneParser 인스턴스
        """
        self.scene_parser = scene_parser
        self.nl_extractor = NLObjectExtractor()

    def build_context(self, nl_command: str) -> Dict:
        """
        자연어 명령과 scene을 분석하여 컨텍스트 생성

        Args:
            nl_command: 자연어 명령

        Returns:
            {
                "scene_info": {
                    "mentioned_objects": {...},
                    "all_objects_summary": {...},
                    "scene_metadata": {...}
                }
            }
        """
        logger.info("Building scene context...")

        # 1. 자연어에서 객체 추출
        nl_objects = self.nl_extractor.extract_objects(nl_command)

        # 2. Scene에서 매칭된 객체 정보 수집
        mentioned_objects = {}
        for nl_obj in nl_objects:
            matched_name = self.scene_parser.match_object_name(nl_obj)
            if matched_name:
                obj_info = self.scene_parser.get_object_info(matched_name)
                if obj_info:
                    mentioned_objects[nl_obj] = {
                        "matched_name": matched_name,
                        "info": obj_info,
                        "nl_reference": nl_obj
                    }

        # 3. Scene 전체 요약
        all_objects = self.scene_parser.get_all_objects()
        scene_metadata = self.scene_parser.parse_scene()["metadata"]

        # 4. 로봇 관절 정보 (있다면)
        robot_joints = self.scene_parser.get_robot_joint_positions()

        context = {
            "scene_info": {
                "mentioned_objects": mentioned_objects,
                "robot_joints": robot_joints,
                "all_objects_count": len(all_objects),
                "scene_metadata": scene_metadata
            }
        }

        logger.info(f"Scene context built: {len(mentioned_objects)} objects matched")
        return context

    def format_for_prompt(self, context: Dict) -> str:
        """
        컨텍스트를 LLM 프롬프트용 문자열로 포맷팅

        Args:
            context: build_context()에서 생성된 컨텍스트

        Returns:
            프롬프트에 삽입할 문자열
        """
        scene_info = context.get("scene_info", {})
        mentioned_objects = scene_info.get("mentioned_objects", {})
        robot_joints = scene_info.get("robot_joints", {})

        sections = []

        # 헤더
        sections.append("# SCENE INFORMATION (from MuJoCo Simulation)")
        sections.append("")
        sections.append("The following objects are present in the current scene:")
        sections.append("")

        # 언급된 객체들
        if mentioned_objects:
            sections.append("## Mentioned Objects")
            sections.append("")

            for nl_ref, obj_data in mentioned_objects.items():
                matched_name = obj_data["matched_name"]
                info = obj_data["info"]

                sections.append(f"### **{matched_name}** (referred as '{nl_ref}')")

                # 위치
                if "position" in info:
                    pos = info["position"]
                    sections.append(f"- Position: ({pos[0]:.3f}m, {pos[1]:.3f}m, {pos[2]:.3f}m)")

                # 회전 (Euler angles)
                if "rotation_euler" in info:
                    rot = info["rotation_euler"]
                    sections.append(f"- Rotation (Euler): ({rot[0]:.1f}°, {rot[1]:.1f}°, {rot[2]:.1f}°)")

                # 질량
                if "mass" in info and info["mass"] > 0:
                    sections.append(f"- Mass: {info['mass']:.2f} kg")

                # 크기
                if "size" in info:
                    size = info["size"]
                    sections.append(f"- Size: {size}")

                # 타입
                if "type" in info:
                    sections.append(f"- Type: {info['type']}")

                sections.append("")

        # 로봇 관절 정보
        if robot_joints:
            sections.append("## Robot Joint Positions")
            sections.append("")
            for joint_name, position in robot_joints.items():
                sections.append(f"- {joint_name}: {position}°")
            sections.append("")

        # Scene 메타데이터
        metadata = scene_info.get("scene_metadata", {})
        if metadata:
            sections.append("## Scene Metadata")
            sections.append(f"- Total bodies: {metadata.get('num_bodies', 0)}")
            sections.append(f"- Total geoms: {metadata.get('num_geoms', 0)}")
            sections.append(f"- Total joints: {metadata.get('num_joints', 0)}")
            if "gravity" in metadata:
                g = metadata["gravity"]
                sections.append(f"- Gravity: ({g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f}) m/s²")
            sections.append("")

        # 중요 지침
        sections.append("## IMPORTANT INSTRUCTIONS")
        sections.append("- Use the ACTUAL positions, masses, and rotations from the scene data above")
        sections.append("- When calculating motion paths, consider object positions and distances")
        sections.append("- If object mass is provided, use SetWorkpieceWeight command in initialization")
        sections.append("- Convert positions from meters to millimeters when needed (1m = 1000mm)")
        sections.append("")

        return "\n".join(sections)

    def enhance_context_with_calculations(self, context: Dict) -> Dict:
        """
        컨텍스트에 계산된 정보 추가 (거리, 방향 등)

        Args:
            context: 기본 컨텍스트

        Returns:
            향상된 컨텍스트
        """
        scene_info = context.get("scene_info", {})
        mentioned_objects = scene_info.get("mentioned_objects", {})

        if len(mentioned_objects) >= 2:
            # 객체 간 거리 계산
            obj_names = list(mentioned_objects.keys())
            distances = {}

            for i, obj1_nl in enumerate(obj_names):
                for obj2_nl in obj_names[i+1:]:
                    obj1_name = mentioned_objects[obj1_nl]["matched_name"]
                    obj2_name = mentioned_objects[obj2_nl]["matched_name"]

                    distance = self.scene_parser.calculate_distance(obj1_name, obj2_name)
                    if distance is not None:
                        key = f"{obj1_nl}_to_{obj2_nl}"
                        distances[key] = distance

            if distances:
                scene_info["calculated_distances"] = distances

        return context


# ==========================================================================
# 3. 편의 함수
# ==========================================================================
def create_scene_context(scene_parser, nl_command: str) -> Dict:
    """
    Scene Context 생성 헬퍼 함수

    Args:
        scene_parser: MuJoCoSceneParser 인스턴스
        nl_command: 자연어 명령

    Returns:
        Scene 컨텍스트 딕셔너리
    """
    builder = SceneContextBuilder(scene_parser)
    context = builder.build_context(nl_command)
    context = builder.enhance_context_with_calculations(context)
    return context


def format_scene_for_prompt(scene_parser, nl_command: str) -> str:
    """
    Scene 정보를 프롬프트용 문자열로 포맷팅

    Args:
        scene_parser: MuJoCoSceneParser 인스턴스
        nl_command: 자연어 명령

    Returns:
        프롬프트에 삽입할 문자열
    """
    builder = SceneContextBuilder(scene_parser)
    context = builder.build_context(nl_command)
    context = builder.enhance_context_with_calculations(context)
    return builder.format_for_prompt(context)
