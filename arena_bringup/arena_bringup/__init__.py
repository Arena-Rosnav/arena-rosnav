import os


def get_arena_ws_dir() -> str:
    _ARENA_WS_DIR = os.environ.get('ARENA_WS_DIR')
    assert _ARENA_WS_DIR is not None, "ARENA_WS_DIR environment variable not set"
    return _ARENA_WS_DIR
