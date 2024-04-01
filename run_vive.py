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
from copy import deepcopy
import socket

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

        self.is_record = False
        self.record_data = {}

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('localhost', 5000))

        
    def calibrate(self):
        origin_device = self.tracking_devices["Tracker_1"]
        self.local_origin = fairmotion_math.invertT(origin_device.get_T())
        self.calibration = False

    # TODO add fps
    def update(self, print=False):
        # if self.calibration:
        #self.calibrate()
        self.tracking_result = [self.local_origin @ self.tracking_devices[key].get_T() for key in self.tracking_devices]
        if print:
            for r in self.tracking_result:
                print("\r" + r, end="")

        if self.is_record:
            self.record_data['data'].append(self.tracking_result)
            self.record_data['record_timestamp'].append(time.time())
        
        # send data to server
        data = {
            'data': self.tracking_result,
            'timestamp': time.time()
        }
        self.client_socket.send(str(data).encode())

    def add_device(self):
        self.vive_tracker_module.update_add_device()
        self.tracking_devices = self.vive_tracker_module.return_selected_devices(self.device_key)
        self.vive_tracker_module.print_discovered_objects()
        
        if 'tracker_keys' in self.record_data:
            self.record_data['tracker_keys'] = list(self.tracking_devices.keys())
            

    def init_record(self):
        self.is_record = True
        self.record_data['fps'] = self.fps
        self.record_data['tracker_keys'] = list(self.tracking_devices.keys())
        self.record_data['data'] = []
        self.record_data['local_origin'] = self.local_origin
        self.record_data['record_start_time'] = datetime.datetime.now()
        self.record_data['record_timestamp'] = []


    def toggle_record(self):
        self.is_record = not self.is_record
        if self.is_record:
            print("init record")
            self.init_record()

    def save(self):
        if len(self.record_data) == 0:
            print("Record is empty. Start recording first!")
            return 
        current_time = datetime.datetime.now()
        time_str = current_time.strftime("%m-%d-%H-%M")

        filename = f"./record/{time_str}.pkl"
        with open(filename, "wb") as file:
            pickle.dump(deepcopy(self.record_data), file)
        print(f"Saving record {filename} with trackers {self.record_data['tracker_keys']}")


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
