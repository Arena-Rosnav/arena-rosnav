#!/bin/bash -i

cd "${ARENA_WS_DIR}/src/arena/arena-rosnav/setup/arena"
sudo apt install -y libopenmpi-dev
$HOME/.local/bin/poetry run poetry install --with planners