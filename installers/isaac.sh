#!/bin/bash -i
cd "${ARENA_WS_DIR}"

source $(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate
