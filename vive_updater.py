import sys
import time
import argparse
from track import ViveTrackerModule
from IPython import embed
from render_argparse import *
from vive_visualizer import ViveTrackerViewer
from fairmotion_vis import camera
from fairmotion_ops import conversions, math as fairmotion_math
from fairmotion_utils import constants
import numpy as np
import datetime
import time
import pickle

# def parse_arguments():
#     parser = argparse.ArgumentParser(description="Vive Tracker Pose Data Display")
#     parser.add_argument("-f", "--frequency", type=float, default=30.0,
#                         help="Frequency of tracker data updates (in Hz). Default: 30 Hz")
#     return parser.parse_args()

# def print_tracker_data(tracker, interval):
#     # Continuously print tracker pose data at the specified interval
#     while True:
#         start_time = time.time()

#         # Get pose data for the tracker device and format as a string
#         pose_data = " ".join(["%.4f" % val for val in tracker.get_pose_euler()])

#         # Print pose data in the same line
#         print("\r" + pose_data, end="")

#         # Calculate sleep time to maintain the desired interval
#         sleep_time = interval - (time.time() - start_time)

#         # Sleep if necessary
#         if sleep_time > 0:
#             time.sleep(sleep_time)

class ViveTrackerUpdater():
    def __init__(self):
        self.vive_tracker_module = ViveTrackerModule()
        self.vive_tracker_module.print_discovered_objects()

        self.fps = 60
        self.device_key = "Tracker"
        self.tracking_devices = self.vive_tracker_module.return_selected_devices(self.device_key)
        self.tracking_result = []

        self.calibration = True
        self.local_origin = constants.eye_T()
        # self.env_offset = conversions.R2T(conversions.Ay2R(conversions.deg2rad(-180)))
        self.env_offset = constants.eye_T()        

        self.is_record = False
        self.record_data = {}
        
    def calibrate(self):
        origin_device = self.tracking_devices["Tracker_1"]
        self.local_origin = fairmotion_math.invertT(origin_device.get_T())
        self.calibration = False

    # TODO add fps
    def update(self, print=False):
        if self.calibration:
            self.calibrate()
        self.tracking_result = [self.env_offset @ self.local_origin @ self.tracking_devices[key].get_T() for key in self.tracking_devices]
        if print:
            for r in self.tracking_result:
                print("\r" + r, end="")

        if self.is_record:
            self.record_data['data'].append(self.tracking_result)
            self.record_data['timestamp'].append(time.time())

    def add_device(self):
        self.vive_tracker_module.update_add_device()
        self.tracking_devices = self.vive_tracker_module.return_selected_devices(self.device_key)
        self.vive_tracker_module.print_discovered_objects()
        
        if 'tracker_keys' in self.record_data:
            self.record_data['tracker_keys'] = self.tracking_devices.keys()
            

    def init_record(self):
        self.is_record = True
        self.record_data['fps'] = self.fps
        self.record_data['tracker_keys'] = self.tracking_devices.keys()
        self.record_data['data'] = []
        self.record_data['local_origin'] = self.local_origin
        self.record_data['record_start_time'] = datetime.datetime.now()
        self.record_data['record_timestamp'] = []


    def toggle_record(self):
        self.is_record = not self.is_record


    def save(self):
        if len(self.record_data) == 0:
            print("Record is empty. Start recording first!")
            return 
        current_time = datetime.datetime.now()
        time_str = current_time.strftime("%m-%d-%H-%M") + ".pkl"

        filename = f"./record/{time_str}.pkl"
        with open(filename, "wb") as file:
            pickle.dump(self.record_data, file, protocol=pickle.HIGHEST_PROTOCOL)


def main(args):
    # Parse command line arguments
    # args = parse_arguments()

    # # Calculate interval based on the specified frequency
    # interval = 1 / args.frequency

    # # Initialize Vive Tracker and print discovered objects
    # v_tracker = ViveTrackerModule()
    # v_tracker.print_discovered_objects()

    # # Print tracker data
    # tracker_1 = v_tracker.devices["tracker_1"]
    # print_tracker_data(tracker_1, interval)

    cam = camera.Camera(
        pos=np.array(args.camera_position),
        origin=np.array(args.camera_origin),
        vup=np.array([0,0,1]), # z-up
        fov=45.0,
    )
    viewer = ViveTrackerViewer(
        v_track_updater=ViveTrackerUpdater(),
        play_speed=args.speed,
        scale=args.scale,
        thickness=args.thickness,
        render_overlay=args.render_overlay,
        hide_origin=args.hide_origin,
        title="Vive Tracker Viewer",
        cam=cam,
        size=(1920, 1280),
    )
    viewer.run()

if __name__ == "__main__":
    args = get_render_args().parse_args()
    main(args)
