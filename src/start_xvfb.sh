#!/bin/bash
# Start Xvfb on display :99 with a resolution of 1920x1080
Xvfb :99 -screen 0 1920x1080x24 &
export DISPLAY=:99

# Start x11vnc to share the virtual display on port 5900
x11vnc -display :99 -N -forever -shared -rfbport 5900 &
exec "$@"