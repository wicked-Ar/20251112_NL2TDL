# mujoco_scene_parser.py
# MuJoCo 시뮬레이션 scene에서 객체 정보를 추출하는 모듈

import os
import logging
import numpy as np
from typing import Dict, List, Optional, Tuple
from difflib import SequenceMatcher
import json

try:
    import mujoco
except ImportError:
    mujoco = None
    logging.warning("MuJoCo not installed. MuJoCo features will be disabled.")

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==========================================================================
# 1. MuJoCo Scene Parser
# ==========================================================================
class MuJoCoSceneParser:
    """
    MuJoCo 시뮬레이션 scene에서 객체 정보를 실시간으로 추출하는 클래스

    주요 기능:
    - MuJoCo 모델 로드 및 데이터 초기화
    - 객체별 위치, 회전, 질량, 크기 정보 추출
    - 자연어 객체 이름 매칭 (fuzzy matching)
    - Scene 스냅샷 생성
    """

    def __init__(self, model_path: Optional[str] = None, model: Optional['mujoco.MjModel'] = None,
                 data: Optional['mujoco.MjData'] = None):
        """
        MuJoCo Scene Parser 초기화

        Args:
            model_path: MuJoCo XML 모델 파일 경로
            model: 직접 제공된 MuJoCo 모델 (선택)
            data: 직접 제공된 MuJoCo 데이터 (선택)
        """
        if mujoco is None:
            raise ImportError("MuJoCo is not installed. Please install with: pip install mujoco")

        self.model = model
        self.data = data
        self.model_path = model_path

        # 모델 로드
        if model_path and model is None:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"MuJoCo model file not found: {model_path}")

            logger.info(f"Loading MuJoCo model from: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            logger.info("MuJoCo model loaded successfully")

        elif model is None:
            raise ValueError("Either model_path or model must be provided")

        # 객체 이름 캐시
        self._object_names_cache = None
        self._body_names_cache = None

    def update_data(self, data: 'mujoco.MjData'):
        """외부에서 시뮬레이션 데이터를 업데이트"""
        self.data = data

    def parse_scene(self) -> Dict:
        """
        현재 scene의 모든 객체 정보를 추출

        Returns:
            {
                "bodies": {...},
                "geoms": {...},
                "joints": {...},
                "metadata": {...}
            }
        """
        logger.info("Parsing MuJoCo scene...")

        scene_data = {
            "bodies": self._parse_bodies(),
            "geoms": self._parse_geoms(),
            "joints": self._parse_joints(),
            "metadata": self._get_scene_metadata()
        }

        logger.info(f"Scene parsed: {len(scene_data['bodies'])} bodies, "
                   f"{len(scene_data['geoms'])} geoms, "
                   f"{len(scene_data['joints'])} joints")

        return scene_data

    def _parse_bodies(self) -> Dict[str, Dict]:
        """모든 body 정보 추출"""
        bodies = {}

        for body_id in range(self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)

            if body_name is None:
                body_name = f"unnamed_body_{body_id}"

            # Body 위치 및 회전
            body_pos = self.data.xpos[body_id].copy()
            body_quat = self.data.xquat[body_id].copy()
            body_euler = self._quat_to_euler(body_quat)

            # Body 질량 (inertia 정보에서)
            body_mass = self.model.body_mass[body_id]

            bodies[body_name] = {
                "id": body_id,
                "position": body_pos.tolist(),
                "quaternion": body_quat.tolist(),
                "rotation_euler": body_euler.tolist(),  # [roll, pitch, yaw] in degrees
                "mass": float(body_mass),
                "type": "body"
            }

        return bodies

    def _parse_geoms(self) -> Dict[str, Dict]:
        """모든 geom 정보 추출"""
        geoms = {}

        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)

            if geom_name is None:
                geom_name = f"unnamed_geom_{geom_id}"

            # Geom 위치 및 회전
            geom_pos = self.data.geom_xpos[geom_id].copy()
            geom_quat = self.data.geom_xmat[geom_id].copy()  # rotation matrix

            # Geom 크기
            geom_size = self.model.geom_size[geom_id].copy()
            geom_type = self.model.geom_type[geom_id]

            geoms[geom_name] = {
                "id": geom_id,
                "position": geom_pos.tolist(),
                "rotation_matrix": geom_quat.tolist(),
                "size": geom_size.tolist(),
                "type": self._get_geom_type_name(geom_type),
                "type_id": int(geom_type)
            }

        return geoms

    def _parse_joints(self) -> Dict[str, Dict]:
        """모든 joint 정보 추출"""
        joints = {}

        for joint_id in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)

            if joint_name is None:
                joint_name = f"unnamed_joint_{joint_id}"

            # Joint 위치 (qpos에서)
            qpos_addr = self.model.jnt_qposadr[joint_id]
            joint_pos = float(self.data.qpos[qpos_addr])

            # Joint 속도 (qvel에서)
            qvel_addr = self.model.jnt_dofadr[joint_id]
            joint_vel = float(self.data.qvel[qvel_addr]) if qvel_addr >= 0 else 0.0

            # Joint 타입
            joint_type = self.model.jnt_type[joint_id]

            joints[joint_name] = {
                "id": joint_id,
                "position": joint_pos,  # angle in radians or position in meters
                "velocity": joint_vel,
                "type": self._get_joint_type_name(joint_type),
                "type_id": int(joint_type)
            }

        return joints

    def _get_scene_metadata(self) -> Dict:
        """Scene 메타데이터 생성"""
        return {
            "model_name": self.model.names.decode('utf-8').split('\x00')[0] if self.model.names else "unknown",
            "num_bodies": self.model.nbody,
            "num_geoms": self.model.ngeom,
            "num_joints": self.model.njnt,
            "timestep": self.model.opt.timestep,
            "gravity": self.model.opt.gravity.tolist()
        }

    def get_object_info(self, object_name: str, object_type: str = "auto") -> Optional[Dict]:
        """
        특정 객체의 정보 반환

        Args:
            object_name: 객체 이름
            object_type: "body", "geom", "joint", "auto" (자동 감지)

        Returns:
            객체 정보 딕셔너리 또는 None
        """
        scene_data = self.parse_scene()

        if object_type == "auto":
            # 모든 타입에서 검색
            for obj_type in ["bodies", "geoms", "joints"]:
                if object_name in scene_data[obj_type]:
                    return scene_data[obj_type][object_name]
            return None
        else:
            obj_dict_key = object_type + "s" if not object_type.endswith("s") else object_type
            return scene_data.get(obj_dict_key, {}).get(object_name)

    def get_all_objects(self) -> List[Dict]:
        """
        모든 객체 리스트 반환 (평탄화)

        Returns:
            [{name: str, type: str, ...}, ...]
        """
        scene_data = self.parse_scene()
        all_objects = []

        for obj_type in ["bodies", "geoms", "joints"]:
            for obj_name, obj_data in scene_data[obj_type].items():
                obj_info = obj_data.copy()
                obj_info["name"] = obj_name
                obj_info["category"] = obj_type[:-1]  # "bodies" -> "body"
                all_objects.append(obj_info)

        return all_objects

    def match_object_name(self, nl_reference: str, threshold: float = 0.6) -> Optional[str]:
        """
        자연어 참조를 실제 객체 이름과 매칭 (fuzzy matching)

        Args:
            nl_reference: 자연어 객체 참조 (예: "용접기", "로봇 A")
            threshold: 유사도 임계값 (0.0 ~ 1.0)

        Returns:
            매칭된 객체 이름 또는 None
        """
        all_objects = self.get_all_objects()

        best_match = None
        best_score = threshold

        nl_lower = nl_reference.lower()

        for obj in all_objects:
            obj_name = obj["name"].lower()

            # 1. 완전 일치 (대소문자 무시)
            if nl_lower == obj_name:
                return obj["name"]

            # 2. 부분 문자열 매칭
            if nl_lower in obj_name or obj_name in nl_lower:
                score = 0.9
                if score > best_score:
                    best_score = score
                    best_match = obj["name"]

            # 3. Fuzzy matching (SequenceMatcher)
            similarity = SequenceMatcher(None, nl_lower, obj_name).ratio()
            if similarity > best_score:
                best_score = similarity
                best_match = obj["name"]

        if best_match:
            logger.info(f"Matched '{nl_reference}' to '{best_match}' (score: {best_score:.2f})")
        else:
            logger.warning(f"No match found for '{nl_reference}'")

        return best_match

    def get_robot_joint_positions(self, robot_body_name: Optional[str] = None) -> Dict[str, float]:
        """
        로봇의 관절 위치 추출

        Args:
            robot_body_name: 로봇 body 이름 (None이면 모든 joint)

        Returns:
            {joint_name: position_in_degrees, ...}
        """
        joints = self._parse_joints()
        robot_joints = {}

        for joint_name, joint_info in joints.items():
            # revolute joint는 radian을 degree로 변환
            if joint_info["type"] in ["hinge", "slide"]:
                if joint_info["type"] == "hinge":
                    position = np.degrees(joint_info["position"])
                else:
                    position = joint_info["position"]  # meters for slide joints

                robot_joints[joint_name] = round(position, 2)

        return robot_joints

    def calculate_distance(self, obj1_name: str, obj2_name: str) -> Optional[float]:
        """
        두 객체 간의 거리 계산

        Args:
            obj1_name: 첫 번째 객체 이름
            obj2_name: 두 번째 객체 이름

        Returns:
            거리 (meters) 또는 None
        """
        obj1 = self.get_object_info(obj1_name)
        obj2 = self.get_object_info(obj2_name)

        if obj1 is None or obj2 is None:
            logger.warning(f"Cannot calculate distance: object not found")
            return None

        pos1 = np.array(obj1.get("position", [0, 0, 0]))
        pos2 = np.array(obj2.get("position", [0, 0, 0]))

        distance = np.linalg.norm(pos2 - pos1)
        return round(float(distance), 3)

    # ========== 유틸리티 메서드 ==========

    @staticmethod
    def _quat_to_euler(quat: np.ndarray) -> np.ndarray:
        """
        Quaternion을 Euler angles로 변환 (ZYX 순서)

        Args:
            quat: [w, x, y, z] quaternion

        Returns:
            [roll, pitch, yaw] in degrees
        """
        w, x, y, z = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        return np.degrees(np.array([roll, pitch, yaw]))

    @staticmethod
    def _get_geom_type_name(geom_type_id: int) -> str:
        """Geom 타입 ID를 이름으로 변환"""
        geom_types = {
            0: "plane",
            1: "hfield",
            2: "sphere",
            3: "capsule",
            4: "ellipsoid",
            5: "cylinder",
            6: "box",
            7: "mesh"
        }
        return geom_types.get(geom_type_id, "unknown")

    @staticmethod
    def _get_joint_type_name(joint_type_id: int) -> str:
        """Joint 타입 ID를 이름으로 변환"""
        joint_types = {
            0: "free",
            1: "ball",
            2: "slide",
            3: "hinge"
        }
        return joint_types.get(joint_type_id, "unknown")

    def export_scene_snapshot(self, filepath: str):
        """Scene 스냅샷을 JSON 파일로 저장"""
        scene_data = self.parse_scene()

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(scene_data, f, indent=2, ensure_ascii=False)

        logger.info(f"Scene snapshot exported to: {filepath}")

    def __repr__(self):
        return f"MuJoCoSceneParser(model={self.model_path or 'provided'}, nbody={self.model.nbody if self.model else 0})"


# ==========================================================================
# 2. Scene Parser Factory (편의 함수)
# ==========================================================================
def create_scene_parser(model_path: str) -> MuJoCoSceneParser:
    """
    Scene Parser 생성 헬퍼 함수

    Args:
        model_path: MuJoCo XML 파일 경로

    Returns:
        MuJoCoSceneParser 인스턴스
    """
    return MuJoCoSceneParser(model_path=model_path)


def create_scene_parser_from_running_sim(model: 'mujoco.MjModel',
                                         data: 'mujoco.MjData') -> MuJoCoSceneParser:
    """
    실행 중인 시뮬레이션에서 Scene Parser 생성

    Args:
        model: MuJoCo 모델
        data: MuJoCo 데이터

    Returns:
        MuJoCoSceneParser 인스턴스
    """
    return MuJoCoSceneParser(model=model, data=data)
