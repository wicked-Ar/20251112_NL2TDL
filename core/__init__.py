# core/__init__.py
# NL2TDL Core Module

"""
NL2TDL Converter Core Module

이 모듈은 자연어를 TDL(Task Description Language)로 변환하는 핵심 기능을 제공합니다.
"""

from .nl2tdl_converter import (
    NL2TDLConverter,
    TDLKnowledgeBase,
    GeminiLLM,
    PromptBuilder,
    TDLProcessor,
    FileManager
)

try:
    from .mujoco_scene_parser import MuJoCoSceneParser, create_scene_parser
    from .scene_context_builder import SceneContextBuilder, NLObjectExtractor
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False

__all__ = [
    'NL2TDLConverter',
    'TDLKnowledgeBase',
    'GeminiLLM',
    'PromptBuilder',
    'TDLProcessor',
    'FileManager',
]

if MUJOCO_AVAILABLE:
    __all__.extend([
        'MuJoCoSceneParser',
        'SceneContextBuilder',
        'NLObjectExtractor',
        'create_scene_parser',
    ])

__version__ = '1.0.0'
