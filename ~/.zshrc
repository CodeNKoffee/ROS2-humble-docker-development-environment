# macOS XQuartz configuration
export PATH="/opt/X11/bin:$PATH"
export DISPLAY=host.docker.internal:0
export QT_QPA_PLATFORM="xcb"

# Add this alias for xhost
alias xhost="/opt/X11/bin/xhost"