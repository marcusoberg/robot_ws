if [ -f "$PIXI_PROJECT_ROOT/install/ros-v2/setup.bash" ]; then
    export COLCON_CURRENT_PREFIX="$PIXI_PROJECT_ROOT/install/ros-v2"
    . "$PIXI_PROJECT_ROOT/install/ros-v2/setup.bash"
    unset COLCON_CURRENT_PREFIX
fi
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
