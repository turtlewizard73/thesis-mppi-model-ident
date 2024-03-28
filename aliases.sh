#! /bin/bash

# aliases
alias g='git status'
alias gf='git fetch'
alias rosbuild="
    cd ~/thesis-mppi-model-ident/workspace &&
    colcon build --symlink-install --cmake-args ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'"

exec "$@"