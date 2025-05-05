#/usr/bin/env bash
distrobox create --name rap-jazzy \
    --image robopaas/rap-jazzy:cuda12.5.0 \
    --additional-flags "\
    --env=WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    --env=XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env=ROS_HOSTNAME=localhost \
    --volume=$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
    --volume=/home/bonsai/Development/rap:/home/ros/rap \
    --security-opt=seccomp=unconfined \
    --security-opt=apparmor=unconfined"
