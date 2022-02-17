#!/usr/bin/env bash
export GST_DEBUG="*:3"
export OPENBLAS_CORETYPE=ARMV8
rm -f /tmp/foo*
python3 main.py &
sleep 3s
gst-zed-rtsp-launch --gst-debug=1 -a 10.18.16.16 -p 5801 shmsrc socket-path=/tmp/foo1 do-timestamp=1 is-live=1 ! video/x-raw, format=BGR, width=672, height=376, framerate=100/1 ! videoconvert ! 'video/x-raw, format=(string)I420' ! nvvidconv ! nvv4l2h265enc bitrate=720000 ! rtph265pay pt=96 name=pay0 &
gst-rtsp-launch -p 5803 "(  v4l2src device=/dev/video0 ! queue !video/x-raw, format=YUY2, width=640, height=480, pixel-aspect-ratio=1/1, framerate=30/1 ! videoconvert ! video/x-raw, format=I420 ! nvvidconv ! nvv4l2h265enc bitrate=720000 ! rtph265pay pt=96 name=pay0 )" &
wait
