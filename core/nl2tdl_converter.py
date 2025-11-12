# nl2tdl_converter.py
# Natural Language to TDL (Task Description Language) Converter
# 사용자의 자연어 명령을 입력받아 TDL 코드를 생성하는 프로그램

import os
import json
import logging
import re
import sys
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import pytz

# Google Generative AI 라이브러리
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold

# ==========================================================================
# 1. 로깅 설정
# ==========================================================================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==========================================================================
# 2. TDL 명령어 지식 베이스 (TDLset.md 기반)
# ==========================================================================
class TDLKnowledgeBase:
    """TDL 명령어 및 문법 규칙을 저장하는 지식 베이스"""

    # TDL 명령어 카테고리별 분류
    COMMANDS = {
        # 제어 흐름
        "control_flow": {
            "If": "If(condition)",
            "Else": "Else()",
            "EndIf": "EndIf()",
            "While": "While(condition)",
            "EndWhile": "EndWhile()",
            "For": "For(variable, start, end)",
            "Next": "Next()",
            "Break": "Break()",
            "Continue": "Continue()",
            "Label": "Label(name)",
            "GoTo": "GoTo(label)",
            "IfGoTo": "IfGoTo(condition, label)",
        },

        # 동작 제어
        "motion": {
            "MoveJoint": "MoveJoint(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None)",
            "MoveLinear": "MoveLinear(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None)",
            "MoveCircular": "MoveCircular(via_pose, target_pose, velocity, acceleration, tool, blending_radius)",
            "MoveBlend": "MoveBlend(pose_list, velocity, acceleration, blending_radius)",
            "AMoveJoint": "AMoveJoint(target_pose, velocity, acceleration, tool, blending_radius)",
            "AMoveLinear": "AMoveLinear(target_pose, velocity, acceleration, tool, blending_radius)",
            "AMoveCircular": "AMoveCircular(via_pose, target_pose, velocity, acceleration, tool, blending_radius)",
            "MotionWait": "MotionWait(handle_id)",
        },

        # 포즈 정의
        "pose": {
            "PosJ": "PosJ(j1, j2, j3, j4, j5, j6)",
            "PosX": "PosX(x, y, z, rx, ry, rz, sol=None)",
            "Trans": "Trans(x, y, z, rx, ry, rz)",
        },

        # I/O 제어
        "io": {
            "SetDigitalOutput": "SetDigitalOutput(port, value)",
            "GetDigitalInput": "GetDigitalInput(port)",
            "WaitForDigitalInput": "WaitForDigitalInput(port, value, timeout_sec)",
            "PulseOutput": "PulseOutput(port, duration_sec, count)",
            "SetAnalogOutput": "SetAnalogOutput(channel, value)",
            "GetAnalogInput": "GetAnalogInput(channel)",
        },

        # 시스템 제어
        "system": {
            "Delay": "Delay(duration_sec)",
            "Stop": "Stop()",
            "End": "End()",
            "PrintLog": "PrintLog(message)",
            "Popup": "Popup(message, type)",
            "Call": "Call(program_id)",
            "ThreadRun": "ThreadRun(fn_name, args=[])",
            "ThreadStop": "ThreadStop(fn_name)",
            "Assign": "Assign(destination, source)",
        },

        # 로봇 설정
        "config": {
            "MakeUserCoordinate": "MakeUserCoordinate(id, pose)",
            "SelectUserCoordinate": "SelectUserCoordinate(id)",
            "SetRefCoord": "SetRefCoord(ref)",
            "SetTool": "SetTool(name_or_params)",
            "SetWorkpieceWeight": "SetWorkpieceWeight(weight, cog)",
            "ToolChange": "ToolChange(id)",
            "SetJointVelocity": "SetJointVelocity(percent)",
            "SetJointAcceleration": "SetJointAcceleration(percent)",
            "SetTaskVelocity": "SetTaskVelocity(mm_per_sec)",
            "SetTaskAcceleration": "SetTaskAcceleration(mm_per_sec2)",
            "SetSingularityHandling": "SetSingularityHandling(mode)",
        },

        # 용접 애플리케이션
        "application": {
            "SpotWeld": "SpotWeld(gun_id, condition_id, sequence_id)",
            "SetArcCondition": "SetArcCondition(condition_id, current, voltage, wire_feed_speed, gas_pre_flow_time, gas_post_flow_time)",
            "ConfigureArcWeaving": "ConfigureArcWeaving(pattern, amplitude, frequency, dwell_time)",
            "ArcOn": "ArcOn()",
            "ArcOff": "ArcOff()",
        },

        # 힘 제어
        "force_control": {
            "StartCompliance": "StartCompliance(stiffness, ref_coord)",
            "ReleaseCompliance": "ReleaseCompliance()",
            "SetDesiredForce": "SetDesiredForce(force, axis, ref_coord)",
        },
    }

    # TDL 구조 템플릿
    TDL_STRUCTURE_TEMPLATE = """GOAL Main_Process()
{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}

GOAL Initialize_Process()
{
    // Initialization commands
}

GOAL Execute_Process()
{
    // Main execution commands
}

GOAL Finalize_Process()
{
    // Finalization commands
}"""

    # COMMAND 정의 템플릿
    COMMAND_DEFINITIONS = """
COMMAND If(condition) { system.flow.if(condition=condition); }
COMMAND Else() { system.flow.else(); }
COMMAND EndIf() { system.flow.endif(); }
COMMAND While(condition) { system.flow.while(condition=condition); }
COMMAND EndWhile() { system.flow.endwhile(); }
COMMAND For(variable, start, end) { system.flow.for(variable=variable, start=start, end=end); }
COMMAND Next() { system.flow.next(); }
COMMAND Break() { system.flow.break(); }
COMMAND Continue() { system.flow.continue(); }
COMMAND Call(program_id) { system.task.call(program_id=program_id); }
COMMAND ThreadRun(fn_name, args=[]) { system.task.thread_run(function=fn_name, args=args); }
COMMAND ThreadStop(fn_name) { system.task.thread_stop(function=fn_name); }
COMMAND Delay(duration_sec) { system.time.delay(duration_sec=duration_sec); }
COMMAND Stop() { system.execution.stop(); }
COMMAND End() { system.execution.end(); }
COMMAND Label(name) { system.flow.define_label(name=name); }
COMMAND GoTo(label) { system.flow.goto(label=label); }
COMMAND IfGoTo(condition, label) { system.flow.if_goto(condition=condition, label=label); }
COMMAND Assign(destination, source) { system.variable.set(destination=destination, source=source); }
COMMAND PrintLog(message) { system.io.print_log(message=message); }
COMMAND Popup(message, type) { system.io.popup(message=message, type=type); }
COMMAND PosJ(j1, j2, j3, j4, j5, j6) { return types.pose.joint(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6); }
COMMAND PosX(x, y, z, rx, ry, rz, sol=None) { return types.pose.cartesian(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, solution=sol); }
COMMAND Trans(x, y, z, rx, ry, rz) { return types.pose.offset(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz); }
COMMAND MakeUserCoordinate(id, pose) { robot.config.set_user_coordinate(id=id, pose=pose); }
COMMAND SelectUserCoordinate(id) { robot.config.select_user_coordinate(id=id); }
COMMAND SetRefCoord(ref) { robot.config.set_reference_coordinate(ref=ref); }
COMMAND SetTool(name_or_params) { robot.config.set_tool_parameters(params=name_or_params); }
COMMAND SetWorkpieceWeight(weight, cog) { robot.config.set_workpiece(weight=weight, center_of_gravity=cog); }
COMMAND ToolChange(id) { robot.tool_changer.execute(id=id); }
COMMAND SetJointVelocity(percent) { robot.config.set_joint_velocity(percent=percent); }
COMMAND SetJointAcceleration(percent) { robot.config.set_joint_acceleration(percent=percent); }
COMMAND SetTaskVelocity(mm_per_sec) { robot.config.set_task_velocity(velocity=mm_per_sec); }
COMMAND SetTaskAcceleration(mm_per_sec2) { robot.config.set_task_acceleration(acceleration=mm_per_sec2); }
COMMAND SetSingularityHandling(mode) { robot.config.set_singularity_mode(mode=mode); }
COMMAND MoveJoint(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None) { motion.execute(type="Joint", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius, sync=synchronized_axes); }
COMMAND MoveLinear(target_pose, velocity, acceleration, tool, blending_radius, synchronized_axes=None) { motion.execute(type="Linear", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius, sync=synchronized_axes); }
COMMAND MoveCircular(via_pose, target_pose, velocity, acceleration, tool, blending_radius) { motion.execute(type="Circular", via=via_pose, target=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius); }
COMMAND MoveBlend(pose_list, velocity, acceleration, blending_radius) { motion.execute(type="Blend", poses=pose_list, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius); }
COMMAND AMoveJoint(target_pose, velocity, acceleration, tool, blending_radius) { return motion.execute_async(type="Joint", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius); }
COMMAND AMoveLinear(target_pose, velocity, acceleration, tool, blending_radius) { return motion.execute_async(type="Linear", pose=target_pose, vel=velocity, acc=acceleration, tool=tool, blend=blending_radius); }
COMMAND AMoveCircular(via_pose, target_pose, velocity, acceleration, tool, blending_radius) { return motion.execute_async(type="Circular", via=via_pose, target=target_pose, vel=velocity, tool=tool, acc=acceleration, blend=blending_radius); }
COMMAND MotionWait(handle_id) { motion.wait(handle=handle_id); }
COMMAND SetDigitalOutput(port, value) { io.digital.set(port=port, value=value); }
COMMAND GetDigitalInput(port) { return io.digital.get(port=port); }
COMMAND WaitForDigitalInput(port, value, timeout_sec) { return io.digital.wait(port=port, value=value, timeout=timeout_sec); }
COMMAND PulseOutput(port, duration_sec, count) { io.digital.pulse(port=port, duration=duration_sec, count=count); }
COMMAND SetAnalogOutput(channel, value) { io.analog.set(channel=channel, value=value); }
COMMAND GetAnalogInput(channel) { return io.analog.get(channel=channel); }
COMMAND SpotWeld(gun_id, condition_id, sequence_id) { application.spot_weld.run(gun=gun_id, condition=condition_id, sequence=sequence_id); }
COMMAND SetArcCondition(condition_id, current, voltage, wire_feed_speed, gas_pre_flow_time, gas_post_flow_time) { application.arc_weld.set_condition(id=condition_id, current=current, voltage=voltage, wire_speed=wire_feed_speed, pre_flow=gas_pre_flow_time, post_flow=gas_post_flow_time); }
COMMAND ConfigureArcWeaving(pattern, amplitude, frequency, dwell_time) { application.arc_weld.configure_weaving(pattern=pattern, amplitude=amplitude, frequency=frequency, dwell=dwell_time); }
COMMAND ArcOn() { application.arc_weld.start(); }
COMMAND ArcOff() { application.arc_weld.stop(); }
COMMAND StartCompliance(stiffness, ref_coord) { robot.force_control.start_compliance(stiffness=stiffness, ref=ref_coord); }
COMMAND ReleaseCompliance() { robot.force_control.release_compliance(); }
COMMAND SetDesiredForce(force, axis, ref_coord) { robot.force_control.set_force(force=force, axis=axis, ref=ref_coord); }
"""

    @classmethod
    def get_all_commands(cls) -> Dict[str, str]:
        """모든 명령어를 평탄화하여 반환"""
        all_commands = {}
        for category, commands in cls.COMMANDS.items():
            all_commands.update(commands)
        return all_commands

    @classmethod
    def get_command_reference(cls) -> str:
        """명령어 레퍼런스 문자열 생성"""
        lines = []
        for category, commands in cls.COMMANDS.items():
            lines.append(f"\n## {category.upper().replace('_', ' ')}")
            for cmd_name, signature in commands.items():
                lines.append(f"  - {signature}")
        return "\n".join(lines)

# ==========================================================================
# 3. Gemini API 기반 LLM 클래스
# ==========================================================================
class GeminiLLM:
    """Google Gemini API를 사용하는 LLM 클래스"""

    def __init__(self, api_key: str, model_name: str = "gemini-2.0-flash-exp"):
        """
        Gemini LLM 초기화

        Args:
            api_key: Google Gemini API 키
            model_name: 사용할 모델 이름
        """
        logger.info(f"Gemini LLM 초기화: {model_name}")
        self.api_key = api_key
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel(model_name)

    def generate(self, prompt: str, temperature: float = 0.1, max_tokens: int = 8192) -> str:
        """
        프롬프트를 기반으로 텍스트 생성

        Args:
            prompt: 입력 프롬프트
            temperature: 생성 온도 (0.0 ~ 1.0)
            max_tokens: 최대 토큰 수

        Returns:
            생성된 텍스트
        """
        try:
            safety_settings = {
                HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_NONE,
            }

            response = self.model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=max_tokens,
                    temperature=temperature
                ),
                safety_settings=safety_settings
            )

            if not response.parts:
                finish_reason = response.candidates[0].finish_reason.name if response.candidates else "UNKNOWN"
                logger.error(f"Gemini API가 콘텐츠를 반환하지 않음. 이유: {finish_reason}")
                return ""

            return response.text.strip()

        except Exception as e:
            logger.error(f"Gemini API 호출 중 오류: {e}")
            return ""

# ==========================================================================
# 4. 프롬프트 생성기
# ==========================================================================
class PromptBuilder:
    """자연어를 TDL로 변환하기 위한 프롬프트 생성 클래스"""

    @staticmethod
    def build_nl2tdl_prompt(nl_command: str, context: Optional[Dict] = None) -> str:
        """
        자연어 명령을 TDL 코드로 변환하는 프롬프트 생성

        Args:
            nl_command: 자연어 명령
            context: 추가 컨텍스트 정보 (scene_info_formatted 포함 가능)

        Returns:
            생성된 프롬프트
        """

        command_reference = TDLKnowledgeBase.get_command_reference()

        # Scene 정보가 context에 있는지 확인
        scene_section = ""
        if context and "scene_info_formatted" in context:
            scene_section = "\n" + context["scene_info_formatted"] + "\n"

        prompt = f"""You are an expert TDL (Task Description Language) code generator. Your task is to convert natural language commands into valid, executable TDL code.
{scene_section}

# TDL SYNTAX AND STRUCTURE

## 1. TDL Program Structure
TDL programs consist of GOAL blocks and COMMAND definitions:

```
GOAL Main_Process()
{{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}}

GOAL Initialize_Process()
{{
    // Setup and initialization commands
}}

GOAL Execute_Process()
{{
    // Main task execution commands
}}

GOAL Finalize_Process()
{{
    // Cleanup and finalization commands
}}
```

## 2. Command Execution Syntax
Commands are executed using SPAWN statements:
```
SPAWN CommandName(arg1, arg2, ...) WITH WAIT;
```

## 3. Available TDL Commands
{command_reference}

## 4. TDL Coding Rules

### Phase Assignment:
- **Initialize_Process**: Setup tasks (configuration, initialization, tool setup)
- **Execute_Process**: Main tasks (motion, I/O operations, main logic)
- **Finalize_Process**: Cleanup tasks (end command, reset states)

### Common Patterns:

**Motion Commands:**
```
SPAWN MoveJoint(PosJ(0, 0, 90, 0, 90, 0), 50, 50, 0, 0) WITH WAIT;
SPAWN MoveLinear(PosX(300, 200, 150, 180, 0, 180), 60, 60, 0, 0) WITH WAIT;
```

**I/O Operations:**
```
SPAWN SetDigitalOutput(1, 1) WITH WAIT;
SPAWN WaitForDigitalInput(11, 1, 10.0) WITH WAIT;
```

**Delays:**
```
SPAWN Delay(2.0) WITH WAIT;
```

**Control Flow:**
```
SPAWN If(condition) WITH WAIT;
    SPAWN MoveJoint(...) WITH WAIT;
SPAWN Else() WITH WAIT;
    SPAWN MoveLinear(...) WITH WAIT;
SPAWN EndIf() WITH WAIT;
```

### Parameter Guidelines:
- **Velocities**: Typically 30-80% for joint motions, 50-150 mm/s for linear
- **Acceleration**: 30-80% or 0-5 for levels
- **Tool ID**: Usually 0 (default tool)
- **Blending radius**: 0 for precise positioning, 1-10 for smooth paths
- **Positions**: Joint angles in degrees, Cartesian positions in mm
- **Timeout**: Typically 5.0-30.0 seconds for wait operations

# YOUR TASK

Convert the following natural language command into a complete TDL program:

**Natural Language Command:**
"{nl_command}"

# OUTPUT REQUIREMENTS

1. Generate ONLY the TDL code - no explanations or markdown
2. Include the Main_Process GOAL that calls the three sub-processes
3. Include all three GOAL blocks (Initialize, Execute, Finalize)
4. Place commands in the appropriate GOAL block based on their purpose
5. Use realistic parameter values
6. Follow proper indentation (4 spaces)
7. All SPAWN commands must end with "WITH WAIT;"
8. Include a header comment with metadata

# OUTPUT FORMAT

```
// TDL Code Generated from Natural Language
// Generated At: <ISO-8601 timestamp in Asia/Seoul timezone>
// Original Command: {nl_command}

GOAL Main_Process()
{{
    SPAWN Initialize_Process() WITH WAIT;
    SPAWN Execute_Process() WITH WAIT;
    SPAWN Finalize_Process() WITH WAIT;
}}

GOAL Initialize_Process()
{{
    // Initialization commands here
}}

GOAL Execute_Process()
{{
    // Main execution commands here
}}

GOAL Finalize_Process()
{{
    SPAWN End() WITH WAIT;
}}
```

Generate the complete TDL code now:
"""
        return prompt

# ==========================================================================
# 5. TDL 코드 후처리 및 검증
# ==========================================================================
class TDLProcessor:
    """생성된 TDL 코드를 처리하고 검증하는 클래스"""

    @staticmethod
    def extract_code(llm_output: str) -> str:
        """LLM 출력에서 TDL 코드만 추출"""
        # 코드 블록 제거
        code = re.sub(r"```(?:tdl)?\s*", "", llm_output)
        code = re.sub(r"```", "", code)

        # GOAL 키워드부터 시작하도록 자르기
        if "//" in code:
            # 주석부터 시작
            lines = code.split('\n')
            for i, line in enumerate(lines):
                if line.strip().startswith('//'):
                    return '\n'.join(lines[i:]).strip()

        if "GOAL" in code:
            idx = code.find("GOAL")
            return code[idx:].strip()

        return code.strip()

    @staticmethod
    def add_metadata_header(tdl_code: str, nl_command: str) -> str:
        """메타데이터 헤더 추가 또는 업데이트"""
        seoul_tz = pytz.timezone("Asia/Seoul")
        timestamp = datetime.now(seoul_tz).isoformat()

        # 기존 헤더 제거
        lines = tdl_code.split('\n')
        code_start = 0
        for i, line in enumerate(lines):
            if line.strip().startswith('GOAL'):
                code_start = i
                break

        # 새 헤더 생성
        header = f"""// TDL Code Generated from Natural Language
// Generated At: {timestamp}
// Original Command: {nl_command}
"""

        return header + '\n' + '\n'.join(lines[code_start:])

    @staticmethod
    def validate_syntax(tdl_code: str) -> Tuple[bool, List[str]]:
        """TDL 코드의 기본 문법 검증"""
        errors = []
        warnings = []

        # GOAL 블록 확인
        required_goals = ["Main_Process", "Initialize_Process", "Execute_Process", "Finalize_Process"]
        for goal in required_goals:
            if f"GOAL {goal}()" not in tdl_code:
                errors.append(f"Missing required GOAL: {goal}")

        # SPAWN 문법 검증
        spawn_lines = re.findall(r'SPAWN\s+[^\n]+', tdl_code)
        for spawn_line in spawn_lines:
            if not spawn_line.strip().endswith('WITH WAIT;'):
                errors.append(f"Invalid SPAWN syntax (missing 'WITH WAIT;'): {spawn_line.strip()}")

        # 괄호 균형 검증
        open_braces = tdl_code.count('{')
        close_braces = tdl_code.count('}')
        if open_braces != close_braces:
            errors.append(f"Unbalanced braces: {open_braces} opening, {close_braces} closing")

        open_parens = tdl_code.count('(')
        close_parens = tdl_code.count(')')
        if open_parens != close_parens:
            errors.append(f"Unbalanced parentheses: {open_parens} opening, {close_parens} closing")

        return (len(errors) == 0, errors)

    @staticmethod
    def add_command_definitions(tdl_code: str, used_commands: List[str] = None) -> str:
        """
        사용된 명령어의 COMMAND 정의를 TDL 코드 끝에 추가

        Args:
            tdl_code: TDL 코드
            used_commands: 사용된 명령어 목록 (None이면 자동 감지)

        Returns:
            COMMAND 정의가 추가된 TDL 코드
        """
        if used_commands is None:
            # SPAWN 문에서 사용된 명령어 자동 추출
            spawn_pattern = r'SPAWN\s+(\w+)\('
            used_commands = list(set(re.findall(spawn_pattern, tdl_code)))

        if not used_commands:
            return tdl_code

        # 전체 COMMAND 정의 가져오기
        all_definitions = TDLKnowledgeBase.COMMAND_DEFINITIONS

        # 사용된 명령어만 필터링
        filtered_definitions = []
        for cmd in used_commands:
            pattern = f"COMMAND {cmd}\\([^{{]*\\)" + r"\s*{[^}]+}"
            match = re.search(pattern, all_definitions)
            if match:
                filtered_definitions.append(match.group(0))

        if filtered_definitions:
            definitions_block = "\n\n// Command Definitions\n" + "\n".join(filtered_definitions)
            return tdl_code + definitions_block

        return tdl_code

# ==========================================================================
# 6. 메인 NL2TDL 변환기
# ==========================================================================
class NL2TDLConverter:
    """자연어를 TDL로 변환하는 메인 클래스"""

    def __init__(self, api_key: str, model_name: str = "gemini-2.0-flash-exp"):
        """
        NL2TDL 변환기 초기화

        Args:
            api_key: Google Gemini API 키
            model_name: 사용할 Gemini 모델
        """
        self.llm = GeminiLLM(api_key, model_name)
        self.processor = TDLProcessor()
        logger.info("NL2TDL Converter 초기화 완료")

    def convert(self, nl_command: str,
                add_command_defs: bool = True,
                context: Optional[Dict] = None,
                scene_parser=None) -> Dict:
        """
        자연어 명령을 TDL 코드로 변환

        Args:
            nl_command: 자연어 명령
            add_command_defs: COMMAND 정의를 추가할지 여부
            context: 추가 컨텍스트 정보
            scene_parser: MuJoCoSceneParser 인스턴스 (선택)

        Returns:
            {
                "success": bool,
                "tdl_code": str,
                "original_command": str,
                "errors": List[str],
                "warnings": List[str],
                "timestamp": str
            }
        """
        logger.info(f"자연어 명령 변환 시작: '{nl_command}'")

        result = {
            "success": False,
            "tdl_code": "",
            "original_command": nl_command,
            "errors": [],
            "warnings": [],
            "timestamp": datetime.now(pytz.timezone("Asia/Seoul")).isoformat()
        }

        try:
            # 1. Scene 정보 추가 (MuJoCo scene parser가 제공된 경우)
            if scene_parser is not None:
                try:
                    from .scene_context_builder import SceneContextBuilder

                    logger.info("MuJoCo scene 정보를 컨텍스트에 추가합니다...")
                    scene_builder = SceneContextBuilder(scene_parser)
                    scene_context = scene_builder.build_context(nl_command)
                    scene_formatted = scene_builder.format_for_prompt(scene_context)

                    # 기존 context와 병합
                    if context is None:
                        context = {}

                    context["scene_info_formatted"] = scene_formatted
                    context["scene_data"] = scene_context

                    logger.info("Scene 정보 추가 완료")
                except Exception as e:
                    logger.warning(f"Scene 정보 추가 중 오류 (무시하고 계속): {e}")

            # 2. 프롬프트 생성
            prompt = PromptBuilder.build_nl2tdl_prompt(nl_command, context)

            # 3. LLM으로 TDL 코드 생성
            logger.info("Gemini API를 통해 TDL 코드 생성 중...")
            raw_output = self.llm.generate(prompt, temperature=0.1)

            if not raw_output:
                result["errors"].append("LLM이 출력을 생성하지 못했습니다.")
                return result

            # 3. 코드 추출
            tdl_code = self.processor.extract_code(raw_output)

            # 4. 메타데이터 헤더 추가
            tdl_code = self.processor.add_metadata_header(tdl_code, nl_command)

            # 5. COMMAND 정의 추가 (옵션)
            if add_command_defs:
                tdl_code = self.processor.add_command_definitions(tdl_code)

            # 6. 문법 검증
            is_valid, errors = self.processor.validate_syntax(tdl_code)

            if not is_valid:
                result["errors"] = errors
                result["warnings"].append("생성된 코드에 문법 오류가 있지만 사용 가능할 수 있습니다.")

            # 7. 결과 반환
            result["success"] = True
            result["tdl_code"] = tdl_code

            logger.info("TDL 코드 생성 완료")
            return result

        except Exception as e:
            logger.error(f"변환 중 오류 발생: {e}", exc_info=True)
            result["errors"].append(f"변환 오류: {str(e)}")
            return result

# ==========================================================================
# 7. 파일 관리
# ==========================================================================
class FileManager:
    """생성된 TDL 파일을 관리하는 클래스"""

    @staticmethod
    def save_tdl(tdl_code: str, nl_command: str, output_dir: str = "output") -> Optional[str]:
        """
        TDL 코드를 파일로 저장

        Args:
            tdl_code: TDL 코드
            nl_command: 원본 자연어 명령
            output_dir: 출력 디렉토리

        Returns:
            저장된 파일 경로 (실패시 None)
        """
        try:
            # 출력 디렉토리 생성
            script_dir = os.path.dirname(os.path.abspath(__file__))
            full_output_dir = os.path.join(script_dir, output_dir)
            os.makedirs(full_output_dir, exist_ok=True)

            # 안전한 파일명 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_name = re.sub(r'[^\w\s-]', '', nl_command)[:30].strip().replace(' ', '_')
            filename = f"nl2tdl_{timestamp}_{safe_name}.tdl"
            filepath = os.path.join(full_output_dir, filename)

            # 파일 저장
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(tdl_code)

            logger.info(f"TDL 파일 저장 완료: {filepath}")
            return filepath

        except Exception as e:
            logger.error(f"파일 저장 중 오류: {e}")
            return None

    @staticmethod
    def save_conversion_log(nl_command: str, tdl_code: str, result: Dict, log_file: str = "conversion_log.json") -> bool:
        """
        변환 로그를 JSON 파일에 저장

        Args:
            nl_command: 원본 자연어 명령
            tdl_code: 생성된 TDL 코드
            result: 변환 결과 딕셔너리
            log_file: 로그 파일 경로

        Returns:
            저장 성공 여부
        """
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            log_path = os.path.join(script_dir, log_file)

            # 기존 로그 읽기
            logs = []
            if os.path.exists(log_path):
                with open(log_path, 'r', encoding='utf-8') as f:
                    logs = json.load(f)

            # 새 로그 추가
            log_entry = {
                "timestamp": result.get("timestamp"),
                "nl_command": nl_command,
                "tdl_code": tdl_code,
                "success": result.get("success"),
                "errors": result.get("errors", []),
                "warnings": result.get("warnings", [])
            }
            logs.append(log_entry)

            # 로그 저장
            with open(log_path, 'w', encoding='utf-8') as f:
                json.dump(logs, f, ensure_ascii=False, indent=2)

            return True

        except Exception as e:
            logger.error(f"로그 저장 중 오류: {e}")
            return False

# ==========================================================================
# 8. CLI 인터페이스 (이 파일이 직접 실행될 때 사용)
# ==========================================================================
def print_banner():
    """프로그램 배너 출력"""
    banner = """
╔══════════════════════════════════════════════════════════════════════╗
║                                                                      ║
║                  NL2TDL Converter - Core Module                      ║
║            Natural Language to TDL Code Generation Engine            ║
║                                                                      ║
║                     Powered by Google Gemini AI                      ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
"""
    print(banner)

def main():
    """메인 함수 (테스트/디버깅용)"""
    print_banner()

    # API 키 설정 (실제 사용시 환경변수나 설정 파일에서 읽어오기)
    API_KEY = "AIzaSyDVz-UCGfPG54Zdinb02gLjSYenG8qMJsc"

    if not API_KEY or API_KEY == "YOUR_API_KEY_HERE":
        print("\n[오류] Gemini API 키가 설정되지 않았습니다.")
        print("코드 내에서 API_KEY 변수를 설정하거나 환경변수를 사용하세요.")
        return

    try:
        # 변환기 초기화
        converter = NL2TDLConverter(api_key=API_KEY)

        # 테스트 명령
        test_command = input("\n자연어 명령을 입력하세요: ").strip()

        if not test_command:
            print("명령이 입력되지 않았습니다.")
            return

        # 변환 실행
        print("\n변환 중...")
        result = converter.convert(test_command, add_command_defs=True)

        # 결과 출력
        print("\n" + "="*70)
        if result["success"]:
            print("✓ 변환 성공!")
            print("="*70)
            print(result["tdl_code"])
            print("="*70)

            # 경고 및 오류 출력
            if result["warnings"]:
                print("\n⚠ 경고:")
                for warning in result["warnings"]:
                    print(f"  - {warning}")

            if result["errors"]:
                print("\n⚠ 문법 검증 오류:")
                for error in result["errors"]:
                    print(f"  - {error}")

            # 파일 저장
            filepath = FileManager.save_tdl(result["tdl_code"], test_command)
            if filepath:
                print(f"\n✓ 파일 저장: {filepath}")

        else:
            print("✗ 변환 실패")
            print("="*70)
            for error in result["errors"]:
                print(f"  - {error}")

    except Exception as e:
        logger.error(f"오류 발생: {e}", exc_info=True)
        print(f"\n[오류] {e}")

if __name__ == "__main__":
    main()
