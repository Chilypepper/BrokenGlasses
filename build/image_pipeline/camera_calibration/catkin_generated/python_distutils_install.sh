#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/peter/brokenGlasses/src/image_pipeline/camera_calibration"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/peter/brokenGlasses/install/lib/python2.7/dist-packages:/home/peter/brokenGlasses/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/peter/brokenGlasses/build" \
    "/usr/bin/python" \
    "/home/peter/brokenGlasses/src/image_pipeline/camera_calibration/setup.py" \
    build --build-base "/home/peter/brokenGlasses/build/image_pipeline/camera_calibration" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/peter/brokenGlasses/install" --install-scripts="/home/peter/brokenGlasses/install/bin"
