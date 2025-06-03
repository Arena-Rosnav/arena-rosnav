#!/bin/bash -i

if ! which uv &>/dev/null ; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

echo 'Enabled optional planners'
