from flask_opencv_streamer.streamer import Streamer as strm


class Streamer:
    def __init__(self, port):
        self.stream = strm(port, False)
        self.stream.start_streaming()

    def write(self, frame):
        self.stream.update_frame(frame)
