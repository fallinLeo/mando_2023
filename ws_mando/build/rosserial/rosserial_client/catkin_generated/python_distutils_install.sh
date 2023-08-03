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

echo_and_run cd "/home/leo/Documents/GitHub/mando_2023/ws_mando/src/rosserial/rosserial_client"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/leo/Documents/GitHub/mando_2023/ws_mando/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/leo/Documents/GitHub/mando_2023/ws_mando/install/lib/python3/dist-packages:/home/leo/Documents/GitHub/mando_2023/ws_mando/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/leo/Documents/GitHub/mando_2023/ws_mando/build" \
    "/usr/bin/python3" \
    "/home/leo/Documents/GitHub/mando_2023/ws_mando/src/rosserial/rosserial_client/setup.py" \
     \
    build --build-base "/home/leo/Documents/GitHub/mando_2023/ws_mando/build/rosserial/rosserial_client" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/leo/Documents/GitHub/mando_2023/ws_mando/install" --install-scripts="/home/leo/Documents/GitHub/mando_2023/ws_mando/install/bin"
