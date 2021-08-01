#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/liam/git/vision_grasp_capstone/ws_capstone/src/mvp_grasp-master/dougsm_helpers"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/liam/git/vision_grasp_capstone/ws_capstone/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/liam/git/vision_grasp_capstone/ws_capstone/install/lib/python2.7/dist-packages:/home/liam/git/vision_grasp_capstone/ws_capstone/build/dougsm_helpers/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/liam/git/vision_grasp_capstone/ws_capstone/build/dougsm_helpers" \
    "/usr/bin/python2" \
    "/home/liam/git/vision_grasp_capstone/ws_capstone/src/mvp_grasp-master/dougsm_helpers/setup.py" \
     \
    build --build-base "/home/liam/git/vision_grasp_capstone/ws_capstone/build/dougsm_helpers" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/liam/git/vision_grasp_capstone/ws_capstone/install" --install-scripts="/home/liam/git/vision_grasp_capstone/ws_capstone/install/bin"
