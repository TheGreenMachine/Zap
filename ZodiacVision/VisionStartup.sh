#!/usr/bin/env bash
python3 main.py &
sleep 3s
gst-zed-rtsp-launch --gst-debug=1 -a 10.18.16.16 -p 5600 shmsrc socket-path=/tmp/foo1 do-timestamp=1 is-live=1 ! video/x-raw, format=RGB, width=672, height=376, framerate=100/1 ! videoconvert ! 'video/x-raw, format=(string)I420' ! nvvidconv ! nvv4l2h265enc bitrate=720000 ! rtph265pay pt=96 name=pay0 &
