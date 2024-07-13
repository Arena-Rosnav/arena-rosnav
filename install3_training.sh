#!/bin/bash -i

cd ~/arena_ws/src/arena/arena-rosnav
poetry run poetry install --with training || \
    (poetry run poetry lock --no-update --with training && poetry run poetry install --with training )