########################################################################
#
# Copyright (c) 2021, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################
import cv2
import sys
import pyzed.sl as sl
from signal import signal, SIGINT

cam = sl.Camera()

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():
    def crop_half(image):
        height, width, channels = image.shape
        # cropped_img = image[0:height, 0:int(width/2)]
        cropped_img = image[0:height, int(width / 2):width]
        return cropped_img
    writer = cv2.VideoWriter('right.mp4',
                             cv2.VideoWriter_fourcc(*'MP4V'),
                             100, (672, 376))

    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 100
    init_params.depth_maximum_distance = 400

    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)

    # path_output = sys.argv[1]
    # recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264)
    # err = cam.enable_recording(recording_param)
    # if err != sl.ERROR_CODE.SUCCESS:
    #     print(repr(status))
    #     exit(1)

    runtime = sl.RuntimeParameters()
    cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 10)
    print("SVO is Recording, use Ctrl-C to stop.")
    frames_recorded = 0
    image_zed = sl.Mat()
    while True:
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS :
            cam.retrieve_image(image_zed, sl.VIEW.RIGHT)
            frames_recorded += 1
            print("Frame count: " + str(frames_recorded), end="\r")
            writer.write(image_zed.get_data())

if __name__ == "__main__":
    main()
