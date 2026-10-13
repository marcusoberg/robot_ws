export PYTHONPYCACHEPREFIX="$PIXI_PROJECT_ROOT/.workspace/python-cache"
export PYTHONPATH="$PIXI_PROJECT_ROOT/install/runtime/python${PYTHONPATH:+:$PYTHONPATH}"
# Shared activation: never source an archived workspace overlay.
if [ -f "$PIXI_PROJECT_ROOT/install/ros/setup.bash" ]; then
    export COLCON_CURRENT_PREFIX="$PIXI_PROJECT_ROOT/install/ros"
    . "$PIXI_PROJECT_ROOT/install/ros/setup.bash"
    unset COLCON_CURRENT_PREFIX
fi
if [ "$(uname -s)" = "Darwin" ]; then
    export DYLD_LIBRARY_PATH="$PIXI_PROJECT_ROOT/install/ros/lib:$CONDA_PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
fi
