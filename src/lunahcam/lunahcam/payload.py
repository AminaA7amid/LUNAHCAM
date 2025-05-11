import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import sys
import os, os.path
import re
import time
from ximea import xiapi
import spidev
import numpy as np
from PIL import Image
import cv2
import csv
from datetime import datetime
import h5py
import subprocess
import threading
import hdf5plugin
import spectral
import shutil
# from threading import Thread, Lock
from pl_interface.msg import PldCmd,CustTime,PldTelem,TargetTemp,Throttled,TelemData
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
os.environ['XIAPI_LOG_LEVEL'] = '0'

from payload_data_injection_config import PayloadConfig
from protocol import Protocol

class Payload(Node, Protocol, PayloadConfig):
    def __init__(_pld):
        super().__init__('payload')
        Protocol.__init__(_pld)
        PayloadConfig.__init__(_pld)
        _pld._allow_undeclared_parameters = True
        _pld.namespace = _pld.get_namespace()
        _pld.ns = _pld.namespace[1:]
        _pld.declare_parameters(
            namespace=_pld.namespace,
            parameters=[
                ('topics.payload', ""),
                ("topics.custom_time", ""),
                ('topics.payload_telem', '/payload_telem'),
                ('topics.target_heater_temp', '/target_heater_temp'),
                (f'viewfinder_config.port', ""),
                ('topics.throttled', '/throttled'),
                ('topics.telemetry_data', '/telem')
            ])
        _pld.custom_time = {'seconds': 0, 'milliseconds': 0}
        _pld.payload_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.payload').value
        _pld.custom_time_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.custom_time').value
        _pld.payload_telem_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.payload_telem').value
        _pld.target_temp_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.target_heater_temp').value
        _pld.throttled_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.throttled').value
        _pld.telem_topic_name = _pld.get_parameter(f'{_pld.namespace}.topics.telemetry_data').value
        _pld.pld_cmd_subscriber = _pld.create_subscription(PldCmd, f'{_pld.payload_topic_name}', _pld.payload_callback, 10)
        _pld.cust_time_subscriber = _pld.create_subscription(CustTime, f'{_pld.custom_time_topic_name}', _pld.get_custom_time_callback, 10)
        _pld.throttled_subscriber = _pld.create_subscription(Throttled, f'{_pld.throttled_topic_name}', _pld.throttled_callback, 10)
        _pld.telem_subscriber = _pld.create_subscription(TelemData, _pld.telem_topic_name, _pld.telem_data_callback, 10)
        _pld.payload_telem_publisher = _pld.create_publisher(PldTelem, _pld.payload_telem_topic_name, 10)
        _pld.target_temp_publisher = _pld.create_publisher(TargetTemp, _pld.target_temp_topic_name, 10)
        _pld.viewfinder_port = _pld.get_parameter(f'{_pld.namespace}.viewfinder_config.viewfinder_port').value
        try:
            subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-fmt-video', 'pixelformat=JPEG,width=2592,height=1944'], check=True)
        except Exception as e:
            pass
        _pld.spi_bus = 0
        _pld.spi_device = 0
        _pld.spi_config()
        _pld.spi_frame = []
        _pld.spi = None
        _pld.cmd_counter = 0
        _pld.exc_cmd_counter = 0
        _pld.packet_sequence_count = 0
        _pld.indexed_files = []
        _pld.img = None
        _pld.hyperspectral_cube = None
        _pld.num_frames_captured = 0
        _pld.hyperspectral_frames = []
        _pld.hf = None
        _pld.extracted_vf_image_index = 0
        _pld.vf_image_size = 0
        _pld.ximea_sensor_temp_sign = False
        _pld.throttled_status = 0
        _pld.ximea_sensor_temp_value = 100
        _pld.extracted_hs_image_index = 0
        _pld.active_mode_telem = 0
        _pld.hs_image_size = 0
        _pld.hs_compressed_image_size = 0
        _pld.vf_cam_status = False
        _pld.ximea_status = False
        _pld.saved_mode = None
        _pld.payload_id_error = 0
        _pld.rpi_temp = 0
        _pld.rpi_temp_sign = True
        _pld.total_seconds = 0 
        _pld.last_exc_cmd = 0
        _pld.current_vf_folder_image_count = 0 
        _pld.ximea_ID = 1
        _pld.rpiCAM_ID = 2
        _pld.hs_file_type = 0 
        _pld.vf_images_dir = "/LUNAHCAM_V1/LUNAHCAM/src/lunahcam"
        _pld.get_last_vf_file_info()
        _pld.hyperspectral_data_dir = "/LUNAHCAM_V1/LUNAHCAM/src/lunahcam"
        _pld.get_last_hs_file_info()
        _pld.default_mode = None
        _pld.mode_lock = threading.Lock()
        _pld.active_mode = None
        _pld.active_mode_thread = None
        _pld.standby_mode = False
        _pld.first_capture_flag = False
        _pld.first_capture_time = None
        _pld.next_capture_time = None
        _pld.mode_directory = "/LUNAHCAM_V1/LUNAHCAM/mode_directory"
        _pld.mode_file = os.path.join(_pld.mode_directory, "lucm_mode.csv")
        _pld.spi_lock = threading.Lock()
        _pld.state = "vf_imaging"
        _pld.mode_map = {
            0x10: _pld.handle_standby,
            0x20: _pld.handle_vf_capture,
            0x30: _pld.handle_ximea_mode,
            0x40: {
                _pld.ximea_ID: _pld.handle_hs_download_mode,
                _pld.rpiCAM_ID: _pld.handle_vf_download_mode
            }
        }
        _pld.payload_telem_timer = _pld.create_timer(1, _pld.publish_payload_telem)
        _pld.abort_event = threading.Event()
        _pld.start_vf_imaging()
        
    def start_vf_imaging(_vf):
        """
        Instead of time.sleep(54), we use a ROS2 timer.
        We'll create a timer that fires exactly once after 54 seconds, then we cancel it inside its callback.
        """
        _vf.get_logger().info("Starting VF imaging in standby for 54 seconds.")
        # Switch to standby (mode=0x10)
        _vf.mode_handler(0x10)
        _vf.standby_mode = True
        _vf.disable_usb()
        # Create a normal timer that fires every 0.1 second, for example:
        # We'll store it, and once 54 seconds pass, we do our “exit standby” logic & cancel the timer.
        # Or we can directly create a 54.0 second timer, then call cancel inside the callback.
        _vf.standby_timer = _vf.create_timer(0.1, _vf._standby_timer_callback)
        _vf.standby_start_time = time.time()
        _vf.standby_duration = 54.0

    def _standby_timer_callback(_vf):
        """
        Called periodically. Once 54 seconds have elapsed, we exit standby mode,
        cancel the timer, and proceed.
        """
        elapsed = time.time() - _vf.standby_start_time
        if elapsed >= _vf.standby_duration:
            _vf.get_logger().info("Standby time complete. Exiting standby mode.")
            _vf.standby_mode = False
            _vf.mode_reader()  # read your CSV to get the mode & freq, then mode_handler
            # Cancel the timer so it won't fire again:
            _vf.standby_timer.cancel()
            _vf.standby_timer = None

    def mode_reader(_mr):
        """
        Reads the saved mode and frequency from the CSV file.
        Expected file formats:
        - Either one row with two columns: e.g. ["0x20", "15"]
        - Or two rows with one column each.
        On success, updates _mr.saved_mode and _mr.vf_frequency_value,
        logs the values, and calls mode_handler().
        On error, falls back to default mode 0x20 and frequency 15.
        """
        try:
            with open(_mr.mode_file, 'r', newline='') as f:
                reader = csv.reader(f)
                rows = list(reader)
            if len(rows) == 1 and len(rows[0]) == 2:
                saved_mode_str = rows[0][0].strip()
                frequency_str = rows[0][1].strip()
            elif len(rows) == 2 and len(rows[0]) >= 1 and len(rows[1]) >= 1:
                saved_mode_str = rows[0][0].strip()
                frequency_str = rows[1][0].strip()
            else:
                raise ValueError("CSV file must have one row with two columns, or two rows with one column each.")
            saved_mode = int(saved_mode_str, 16)
            frequency = int(frequency_str)
            _mr.saved_mode = (saved_mode)
            _mr.vf_frequency_value = frequency
            # _mr.get_logger().info(
            #     f"Mode file read: saved_mode = {saved_mode} (hex: {hex(saved_mode)}), "
            #     f"vf_frequency_value = {frequency}"
            # )
            _mr.mode_handler(saved_mode)
        except Exception as e:
            # _mr.get_logger().warning(
            #     f"Error reading mode file '{_mr.mode_file}': {e}. Falling back to default mode (0x20) with frequency 15."
            # )
            _mr.saved_mode = 0x20
            _mr.vf_frequency_value = 15
            _mr.mode_handler(_mr.saved_mode)

          
    def mode_handler(_mh, mode_code, payload_id=None, *args, **kwargs):
        """
        Updated mode_handler using a threading.Event for abort signaling.
        States:
          0x10: standby
          0x20: vf_imaging
          0x30: hs_imaging
          0x40: download
        Steps:
          1. Signal the current mode thread to abort.
          2. If a mode thread is running (and it is not the current thread), join it.
          3. Clear the abort event.
          4. Validate and update the new mode and state, then publish telemetry.
          5. For download (0x40) mode, select the proper handler based on payload_id.
          6. Spawn the new modes thread.
        """
        try:
            with _mh.mode_lock:
                # Signal the current thread to stop work.
                _mh.abort_event.set()
                # If there is an active mode thread and it is not the current thread, wait for it to finish.
                if _mh.active_mode_thread and _mh.active_mode_thread != threading.current_thread():
                    _mh.active_mode_thread.join(timeout=1.0)
                # Clear the abort event for the new mode.
                _mh.abort_event.clear()
                # Validate the mode code.
                if mode_code not in _mh.mode_map:
                    raise ValueError(f"Invalid mode: {hex(mode_code)}")
                if mode_code == 0x10:
                    _mh.state = "standby"
                elif mode_code == 0x20:
                    _mh.state = "vf_imaging"
                elif mode_code == 0x30:
                    _mh.state = "hs_imaging"
                elif mode_code == 0x40:
                    _mh.state = "download"
                _mh.active_mode = mode_code
                _mh.active_mode_telem = mode_code
                # For download mode, require a valid payload_id and select the appropriate handler.
                if mode_code == 0x40:
                    dl_map = _mh.mode_map[0x40]
                    if not isinstance(dl_map, dict):
                        raise ValueError("mode_map[0x40] must be a dict keyed by payload_id")
                    if payload_id is None:
                        raise ValueError("Payload ID is required for download mode (0x40)")
                    handler = dl_map.get(payload_id)
                    if handler is None:
                        raise ValueError(f"Invalid payload ID {hex(payload_id)} for download mode")
                else:
                    handler = _mh.mode_map[mode_code]

            # Spawn the new mode thread.
            _mh.active_mode_thread = threading.Thread(
                target=handler,
                args=args,
                kwargs=kwargs,
                daemon=True
            )
            _mh.active_mode_thread.start()
        except Exception as e:
            # _mh.get_logger().error(f"mode_handler error: {str(e)}")
            if _mh.saved_mode is not None and _mh.saved_mode != mode_code:
                _mh.mode_handler(_mh.saved_mode)
            else:
                pass
                # _mh.get_logger().error("No valid saved mode available for fallback.")

    def telem_data_callback(_utd, msg):
        _utd.rpi_temp_sign = msg.rpi_cpu_sign
        _utd.rpi_temp = (int(msg.rpi_cpu)/100)

    def throttled_callback(_tc, msg):
        """
        Callback for handling the throttled message.
        Enters standby mode if throttled is non-zero, exits if throttled is zero.
        """
        _tc.throttled_status = msg.throttled

    def handle_standby(_hs):
        try:
            _hs.active_mode_telem = 0x10
            # Remove or avoid any unconditional resetting
            if (_hs.throttled_status != 0 or (_hs.rpi_temp >=85 and _hs.rpi_temp_sign)) and not _hs.standby_mode:
                _hs.standby_mode = True
                _hs.active_mode_telem = 0x10
                _hs.cleanup_spi()
            elif _hs.throttled_status == 0 and _hs.standby_mode:
                _hs.standby_mode = False
                # log that you're exiting standby mode if desired
        except Exception as e:
            pass

    def cleanup_spi(_cl):
        """Clean up any open SPI connections."""
        if hasattr(_cl, 'spi') and _cl.spi:
            try:
                _cl.spi.close()
                # print("SPI device cleaned up and closed.")
            except Exception as e:
                pass
                # print(f"Error during SPI cleanup: {e}")
        else:
            # print("No active SPI connection to clean up.")
            pass

    def get_last_vf_file_info(_gv):
        """
        Retrieves the size and index of the last VF image file stored.
        """
        try:
            # Ensure the directory exists
            if not os.path.exists(_gv.vf_images_dir):
                # print(f"VF images directory {_gv.vf_images_dir} does not exist.")
                return

            files = [f for f in os.listdir(_gv.vf_images_dir) if re.match(r"view_\d+\.bmp", f)]

            if not files:
                pass
                return

            files.sort(key=lambda x: int(re.search(r"(\d+)", x).group(1)))
            _gv.current_vf_folder_image_count = len([name for name in os.listdir(_gv.vf_images_dir) if os.path.isfile(os.path.join(_gv.vf_images_dir, name))])
            # Get the last file
            last_file = files[-1]
            _gv.extracted_vf_image_index = int(re.search(r"(\d+)", last_file).group(1))
            last_file_path = os.path.join(_gv.vf_images_dir, last_file)
            _gv.vf_image_size = int(os.path.getsize(last_file_path) / 1024)
            # Log the results
            # print(f"Last VF image file: {last_file}, Index: {_gv.extracted_vf_image_index}, Size: {_gv.vf_image_size} KB")

        except Exception as e:
            pass
            # print(f"Error retrieving last VF image file info: {e}")

    def get_last_hs_file_info(_gh):
        """
        Retrieves the latest hyperspectral image index, the size of its raw file (in MB),
        and, if present, the size of the corresponding compressed file (in MB).
        The raw and compressed files are expected to be inside a folder named "hyperspectral_<index>".
        """
        try:
            base_dir = _gh.hyperspectral_data_dir
            if not os.path.exists(base_dir):
                return

            # List all subdirectories that match the pattern
            dirs = [
                d for d in os.listdir(base_dir)
                if os.path.isdir(os.path.join(base_dir, d)) and re.match(r"hyperspectral_\d+", d)
            ]
            if not dirs:
                return

            # Find the last (highest-index) subdirectory
            dirs.sort(key=lambda d: int(re.search(r"hyperspectral_(\d+)", d).group(1)))
            last_dir = dirs[-1]

            # Extract the numeric index from the directory name
            _gh.extracted_hs_image_index = int(re.search(r"hyperspectral_(\d+)", last_dir).group(1))

            # Construct the raw file path
            raw_file_path = os.path.join(base_dir, last_dir, f"hyperspectral_{_gh.extracted_hs_image_index}.h5")

            if os.path.exists(raw_file_path):
                _gh.hs_image_size = int(os.path.getsize(raw_file_path) / (1024 * 1024))
            else:
                _gh.hs_image_size = 0
            bz2_file_path = raw_file_path.replace(".h5", "_bz2.h5")
            if os.path.exists(bz2_file_path):
                _gh.hs_compressed_image_size = int(os.path.getsize(bz2_file_path) / (1024 * 1024))
            else:
                _gh.hs_compressed_image_size = 0

        except Exception as e:
            _gh.get_logger().error(f"Error retrieving hyperspectral file info: {e}")


                
    def handle_vf_capture(_vf):
        """Handles VF imaging mode using a ROS 2 timer instead of a while loop."""
        _vf.get_logger().info("Entering VF imaging mode (VF capture).")
        # Ensure any previous abort flags are cleared
        _vf.abort_event.clear()

        # Get the current time in seconds
        current_time = _vf.custom_time["seconds"] + _vf.custom_time["milliseconds"] / 1000.0

        # If this is the first capture, capture immediately and schedule the next capture time.
        if _vf.first_capture_time is None:
            _vf.first_capture_time = current_time
            # Capture image immediately
            _vf.vf_camera_capture()
            image_index = _vf.extracted_vf_image_index
            if image_index > 0:
                _vf.get_vf_img(image_index)
            # Set next capture time to current time plus the desired interval (e.g. vf_frequency_value minutes)
            _vf.next_capture_time = current_time + (_vf.vf_frequency_value * 60)
        # Start the periodic VF capture timer (runs every second)
        _vf.vf_capture_timer = _vf.create_timer(1.0, _vf._vf_capture_callback)


    def _vf_capture_callback(_vf):
        """Timer callback to periodically check whether to capture a VF image."""
        if _vf.abort_event.is_set() or _vf.active_mode != 0x20:
            _vf.get_logger().info("VF capture timer aborted or mode changed; canceling timer.")
            _vf.vf_capture_timer.cancel()
            return
        try:
            current_time = _vf.custom_time["seconds"] + _vf.custom_time["milliseconds"] / 1000.0
            # Check if it's time to capture an image
            if current_time >= _vf.next_capture_time:
                _vf.vf_camera_capture()
                # Schedule the next capture time
                _vf.next_capture_time = current_time + (_vf.vf_frequency_value * 60)
                # Start image transmission if an image was captured
                image_index = _vf.extracted_vf_image_index
                if image_index > 0:
                    _vf.get_vf_img(image_index)
        except Exception as e:
            _vf.get_logger().error(f"Error in VF capture callback: {str(e)}")


    def vf_camera_capture(_vf):
        try:
            device = '/dev/video0'
            desired_width = 2592
            desired_height = 1944
            save_dir = _vf.vf_images_dir
            os.makedirs(save_dir, exist_ok=True)
            vf_image_index = 1
            while True:
                _vf.vf_image_path = os.path.join(save_dir, f"view_{vf_image_index}.bmp")
                if not os.path.exists(_vf.vf_image_path):
                    break
                vf_image_index += 1
            cap = cv2.VideoCapture(device)
            if not cap.isOpened():
                return None
                pass
            _vf.vf_cam_status = True
            _vf.publish_payload_telem()
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                _vf.vf_cam_status = False
                return None
            cv2.imwrite(_vf.vf_image_path, frame)
            # , [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            cap.release()
            _vf.vf_cam_status = False
            _vf.get_last_vf_file_info()
            _vf.active_mode_telem= 0x20
            _vf.publish_payload_telem()
        except Exception as e:
            pass
            return None
        
    # def vf_camera_capture(_vf):
    #     try:
    #         device = '/dev/video0'
    #         desired_width = 2592
    #         desired_height = 1944
    #         save_dir = _vf.vf_images_dir
    #         os.makedirs(save_dir, exist_ok=True)
    #         vf_image_index = 1
    #         while True:
    #             _vf.vf_image_path = os.path.join(save_dir, f"view_{vf_image_index}.raw")
    #             if not os.path.exists(_vf.vf_image_path):
    #                 break
    #             vf_image_index += 1
    #         cap = cv2.VideoCapture(device)
    #         _vf.vf_status = True
    #         cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    #         cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    #         ret, frame = cap.read()
    #         if not ret:
    #             cap.release()
    #             _vf.vf_status = False
    #             return None
    #         frame_uint8 = np.array(frame, dtype=np.uint8)
    #         frame_uint8.tofile(_vf.vf_image_path)
    #         cap.release()
    #         _vf.vf_status = False
    #         _vf.get_last_vf_file_info()
    #         _vf.active_mode_telem= 0x20

    #         # print(f"Captured raw image saved to {_vf.vf_image_path}")
    #     except Exception as e:
    #         pass
    #         # print(f"Error capturing image: {e}")
    #         return None

    def payload_callback(_cb, msg):
        try:
            _cb.payload_command = msg.data[_cb.idxCMD_ID]
            if _cb.state == "standby":
                match _cb.payload_command:
                    case _cb.cmdTURN_0N_WIFI :
                        _cb.turn_on_wifi()
                    case _cb.cmdTURN_OFF_WIFI:
                        _cb.turn_off_wifi()
                    case _cb.cmdLUCM_MODE:
                        _cb.last_exc_cmd = 23
                        mode = msg.data[1]
                        vf_frequency_value = msg.data[2]
                        _cb.lucm_mode(mode, vf_frequency_value)
                        _cb.exc_cmd_counter += 1
                        _cb.publish_payload_telem()
                    case _cb.cmdSET_TEMP:
                        _cb.last_exc_cmd = 22
                        _cb.set_target_temp(msg.data[2])
                        _cb.exc_cmd_counter += 1
                        _cb.publish_payload_telem()
                    case _:
                        pass
                return
            _cb.payload_id = msg.data[_cb.idxPAYLOAD_ID]
            match _cb.payload_command:
                case _cb.cmdCAPTURE_IMAGE:
                    match _cb.payload_id:
                        case _cb.ximea_ID:
                            _cb.last_exc_cmd = 17
                            exp_time_bytes = msg.data[2:6]
                            _cb.ximea_exp_time = int.from_bytes(exp_time_bytes, byteorder='big', signed=False)
                            _cb.ximea_target_fps = msg.data[6]
                            gain_sign = msg.data[7]
                            gain_bytes = msg.data[8:10]
                            _cb.ximea_gain = int.from_bytes(gain_bytes, byteorder='big', signed=False)
                            match gain_sign:
                                case 0:
                                    pass
                                case 255:
                                    _cb.ximea_gain = -_cb.ximea_gain
                                case _:
                                    pass
                            _cb.ximea_trigger_src = msg.data[10]
                            xi_prm_width_bytes = msg.data[11:13]
                            _cb.ximea_xi_prm_width = int.from_bytes(xi_prm_width_bytes, byteorder='big', signed=False)
                            xi_prm_height_bytes = msg.data[13:15]
                            _cb.ximea_xi_prm_height = int.from_bytes(xi_prm_height_bytes, byteorder='big', signed=False)
                            prm_offset_x_bytes = msg.data[15:17]
                            _cb.ximea_prm_offset_x = int.from_bytes(prm_offset_x_bytes, byteorder='big', signed=False)
                            prm_offset_y_bytes = msg.data[17:19]
                            _cb.ximea_prm_offset_y = int.from_bytes(prm_offset_y_bytes, byteorder='big', signed=False)
                            _cb.ximea_img_format = msg.data[19]
                            no_of_frames_bytes = msg.data[20:22]
                            _cb.ximea_no_of_frames = int.from_bytes(no_of_frames_bytes, byteorder='big', signed=False)
                            _cb.ximea_capture_duration = msg.data[22]
                            _cb.mode_handler(0x30, payload_id=_cb.ximea_ID)
                        case _cb.rpiCAM_ID:
                            _cb.last_exc_cmd = 17
                            _cb.mode_handler(0x20, payload_id=_cb.rpiCAM_ID)
                        case _:
                            _cb.payload_id_error_callback()
                case _cb.cmdGET_IMAGE:
                    image_index_bytes = msg.data[2:4]
                    image_index = int.from_bytes(image_index_bytes, byteorder='big', signed=False)
                    seq_num_bytes = msg.data[4:8]
                    seq_num = int.from_bytes(seq_num_bytes, byteorder='big', signed=False)
                    window_bytes = msg.data[8:10]
                    window_num = int.from_bytes(window_bytes, byteorder='big', signed=False)
                    _cb.hs_file_type = msg.data[10]
                    match _cb.payload_id:
                        case _cb.ximea_ID:
                            _cb.last_exc_cmd = 18
                            _cb.mode_handler(
                                0x40,
                                payload_id=_cb.ximea_ID,
                                image_index=image_index,
                                seq_num=seq_num,
                                window_num=window_num
                            )
                        case _cb.rpiCAM_ID:
                            _cb.last_exc_cmd = 18
                            _cb.mode_handler(
                                0x40,
                                payload_id=_cb.rpiCAM_ID,
                                image_index=image_index,
                                seq_num=seq_num,
                                window_num=window_num
                            )
                        case _:
                            _cb.payload_id_error_callback()
                case _cb.cmdDELETE_IMAGE:
                    delete_image_flag = msg.data[2]
                    del_image_index = msg.data[3:5]
                    del_image_index_value = int.from_bytes(del_image_index, byteorder='big', signed=False)
                    match _cb.payload_id:
                        case _cb.ximea_ID:
                            _cb.last_exc_cmd = 20
                            _cb.delete_hs_img(delete_flag=delete_image_flag, image_index=del_image_index_value)
                        case _cb.rpiCAM_ID:
                            _cb.last_exc_cmd = 20
                            _cb.delete_vf_img(delete_flag=delete_image_flag, image_index=del_image_index_value)
                        case _:
                            _cb.payload_id_error_callback()
                    _cb.exc_cmd_counter += 1
                    _cb.publish_payload_telem()
                case _cb.cmdRUN_AI:
                    _cb.run_ai()
                    _cb.exc_cmd_counter += 1
                    _cb.publish_payload_telem()
                case _cb.cmdSET_TEMP:
                    _cb.last_exc_cmd = 22
                    _cb.set_target_temp(msg.data[2])
                    _cb.exc_cmd_counter += 1
                    _cb.publish_payload_telem()
                case _cb.cmdLUCM_MODE:
                    _cb.last_exc_cmd = 23
                    mode = msg.data[1]
                    vf_frequency_value = msg.data[2]
                    _cb.lucm_mode(mode, vf_frequency_value)
                    _cb.exc_cmd_counter += 1
                    _cb.publish_payload_telem()
                case _cb.cmdTURN_0N_WIFI:
                    _cb.turn_on_wifi()
                case _cb.cmdTURN_OFF_WIFI:
                    _cb.turn_off_wifi()
                case _:
                    _cb.default_callback()
        except Exception as e:
            pass
            # _cb.get_logger().error(f"payload_callback error: {str(e)}")
    def payload_id_error_callback(_id):
        # print("Payload ID error: Unknown payload ID.")
        _id.payload_id_error = 0x21

    def handle_ximea_mode(_xi):
        try:
            _xi.enable_usb()
            _xi.active_mode_telem = 0x30
            _xi.Open_Ximea_Connection()
            max_duration = 60
            start_time = time.time()
            frames_captured = 0
            while frames_captured < _xi.ximea_no_of_frames:
                if (time.time() - start_time) > max_duration:
                    break

                _xi.Capture_Ximea_Image()
                _xi.ximea_sensor_temp_value = _xi.cam.get_sensor_board_temp()
                frames_captured += 1
            raw_filename = _xi.Save_Hyperspectral_Data()
            if raw_filename:
                _xi.compress_file_bz2(raw_filename)
            else:
                _xi.get_logger().error("Failed to save raw hyperspectral data.")
            _xi.Close_Ximea_Connection()
            _xi.active_mode_telem = 0x30
            _xi.exc_cmd_counter += 1

        except Exception as e:
            _xi.get_logger().error(f"Error in handle_ximea_mode: {e}")
            _xi.disable_usb()
            _xi.mode_handler(_xi.saved_mode)
        finally:
            _xi.disable_usb()
            _xi.ximea_sensor_temp_value = 100
            _xi.mode_handler(_xi.saved_mode)
            _xi.hyperspectral_frames = []

    def handle_vf_download_mode(_dl, image_index, seq_num, window_num):
        """Handles VF download mode."""
        _dl.standby_mode = False
        try:
            _dl.active_mode_telem = 0x40
            _dl.packet_sequence_count = 0
            # _dl.get_logger().info(f'Entering VF download mode for image index {image_index}, sequence number {seq_num}, window number {window_num}')
            if window_num == 0:
                # Send the entire VF image
                _dl.get_vf_img(image_index)
            else:
                # Send frames starting from seq_num until seq_num + window_num
                _dl.get_vf_img_partial(image_index, seq_num, window_num)
            _dl.exc_cmd_counter += 1
            _dl.packet_sequence_count = 0
            # _dl.get_logger().info('VF download complete.')
        except Exception as e:
            # _dl.get_logger().error(f'Error in handle_vf_download_mode: {e}')
            _dl.cleanup_spi()
            _dl.mode_handler(_dl.saved_mode)
        finally:
            _dl.cleanup_spi()
            _dl.mode_handler(_dl.saved_mode)

    def handle_hs_download_mode(_dl, image_index, seq_num, window_num):
        """Handles hyperspectral download mode."""
        _dl.standby_mode = False
        try:
            _dl.active_mode_telem = 0x40
            _dl.packet_sequence_count = 0        
            if window_num == 0:
                # Send the entire hyperspectral image
                _dl.get_hs_img(image_index)
            else:
                # Send frames starting from seq_num until seq_num + window_num
                _dl.get_hs_img_partial(image_index, seq_num, window_num)
            _dl.exc_cmd_counter += 1
            _dl.packet_sequence_count = 0        
        except Exception as e:
            _dl.cleanup_spi()
            _dl.mode_handler(_dl.saved_mode)
        finally:
            _dl.cleanup_spi()
            _dl.mode_handler(_dl.saved_mode)

    def get_hs_img_partial(_gh, hs_image_index, seq_num, window_num):
        """
        Retrieves and sends a partial hyperspectral image (raw or compressed) via SPI,
        starting from chunk seq_num and sending window_num chunks.

        The numbering scheme is:
        - 1 => raw file (hyperspectral_{index}.h5)
        - 2 => compressed file (hyperspectral_{index}_bz2.h5)
        stored in _gh.hs_file_type beforehand.
        """
        _gh.standby_mode = False
        try:
            # If not already in download mode, set it
            if _gh.active_mode != 0x40:
                _gh.mode_handler(0x40)

            base_dir = _gh.hyperspectral_data_dir
            folder_name = f"hyperspectral_{hs_image_index}"

            # Decide which file to open based on the numbering scheme
            if getattr(_gh, 'hs_file_type', 1) == 2:
                # If user said "2 => compressed"
                h5_filename = f"hyperspectral_{hs_image_index}_bz2.h5"
            else:
                # Default or "1 => raw"
                h5_filename = f"hyperspectral_{hs_image_index}.h5"

            h5_path = os.path.join(base_dir, folder_name, h5_filename)

            # Open the file (raw or compressed) and read as binary
            with open(h5_path, "rb") as f:
                binary_data = f.read()
            if binary_data is None:
                return

            slice_size = 2048
            total_bytes = len(binary_data)
            num_chunks = (total_bytes + slice_size - 1) // slice_size  # Round up

            # Calculate start and end chunk indices
            start_chunk = seq_num
            end_chunk = min(seq_num + window_num, num_chunks)

            for i in range(start_chunk, end_chunk):
                if _gh.abort_event.is_set():
                    break

                # Determine if it's first, middle, or final chunk
                if i == start_chunk:
                    frame_position = 'first'
                elif i == end_chunk - 1:
                    frame_position = 'final'
                else:
                    frame_position = 'middle'

                start_idx = i * slice_size
                end_idx = start_idx + slice_size
                data_chunk = binary_data[start_idx:end_idx]

                # Pad the final chunk if necessary
                if frame_position == 'final' and len(data_chunk) < slice_size:
                    padding_length = slice_size - len(data_chunk)
                    data_chunk += bytes([0] * padding_length)

                _gh.build_spi_frame(data_chunk, frame_position)
                _gh.toggle_spi()
                time.sleep(0.05)

        except Exception as e:
            pass
        finally:
            _gh.cleanup_spi()


    def get_vf_img_partial(_gi, vf_image_index, seq_num, window_num):
        """
        Retrieves and sends a partial VF image via SPI, starting from seq_num and ending at seq_num + window_num.
        """
        _gi.standby_mode = False
        try:
            if _gi.active_mode != 0x40:
                _gi.mode_handler(0x40)        
            save_dir = _gi.vf_images_dir
            bmp_image_path = os.path.join(save_dir, f"view_{vf_image_index}.bmp")
            if not os.path.exists(bmp_image_path):
                pass
                return
            with open(bmp_image_path, "rb") as f:
                binary_data = f.read()
            # Step 3: Slice the binary data into chunks of 2048 bytes
            chunk_size = 2048
            total_bytes = len(binary_data)
            num_chunks = (total_bytes + chunk_size - 1) // chunk_size  # Round up

            # Calculate start and end chunk indices
            start_chunk = seq_num
            end_chunk = min(seq_num + window_num, num_chunks)

            # Step 4: Loop through the chunks and send each as an SPI frame
            for i in range(start_chunk, end_chunk):
                if _gi.abort_event.is_set():
                    break
                frame_position = 'middle'
                start_idx = i * chunk_size
                end_idx = start_idx + chunk_size

                if i == start_chunk:
                    frame_position = 'first'
                elif i == end_chunk - 1:
                    frame_position = 'final'

                # Extract the current chunk
                data_chunk = binary_data[start_idx:end_idx]

                # If this is the final frame and the data chunk is less than 2048 bytes, pad with zeros
                if frame_position == 'final' and len(data_chunk) < chunk_size:
                    padding_length = chunk_size - len(data_chunk)
                    data_chunk += bytes([0] * padding_length)

                # Build the SPI frame for the current chunk
                _gi.build_spi_frame(data_chunk, frame_position)

                # Send the SPI frame
                _gi.toggle_spi()
                time.sleep(0.05)

        except Exception as e:
            pass
        finally:
            # Clean up resources
            _gi.cleanup_spi()


    def Open_Hyperspectral_Data(_oh, open_filename):
        """
        Opens an HDF5 file and loads the hyperspectral dataset.

        :param open_filename: Path to the HDF5 file to open.
        :return: The dataset as a NumPy array, or None if an error occurs.
        """
        try:
            with h5py.File(open_filename, 'r') as open_hf:
                if 'dataset' in open_hf:
                    dataset = open_hf['dataset'][:]  # Load the dataset as a NumPy array
                    return dataset
                else:
                    return None
        except Exception as e:
            return None       
         
    def get_hs_img(_gh, hs_image_index, frames_per_second=14):
        """
        Retrieves and sends hyperspectral data via SPI. Uses the numbering scheme:
        - 1 => raw file (hyperspectral_{index}.h5)
        - 2 => compressed file (hyperspectral_{index}_bz2.h5)

        The user's choice (1 or 2) is stored in _gh.hs_file_type beforehand.
        """
        _gh.standby_mode = False
        try:
            # Ensure mode is set only once at the beginning
            if _gh.active_mode != 0x40:
                _gh.mode_handler(0x40)  # Set to hyperspectral mode

            # Decide which file to open based on the numbering scheme
            base_dir = _gh.hyperspectral_data_dir
            folder_name = f"hyperspectral_{hs_image_index}"
            if getattr(_gh, 'hs_file_type', 1) == 2:
                h5_filename = f"hyperspectral_{hs_image_index}_bz2.h5"
            else:
                h5_filename = f"hyperspectral_{hs_image_index}.h5"

            h5_path = os.path.join(base_dir, folder_name, h5_filename)

            # Open the file (raw or compressed) and read as binary
            with open(h5_path, "rb") as f:
                binary_data = f.read()
            if binary_data is None:
                return

            slice_size = 2048
            total_bytes = len(binary_data)
            num_chunks = (total_bytes + slice_size - 1) // slice_size  # Round up

            for i in range(num_chunks):
                if _gh.abort_event.is_set():
                    break

                if i == 0:
                    frame_position = 'first'
                elif i == num_chunks - 1:
                    frame_position = 'final'
                else:
                    frame_position = 'middle'

                start_idx = i * slice_size
                end_idx = start_idx + slice_size
                data_chunk = binary_data[start_idx:end_idx]

                # Pad the final chunk if necessary
                if frame_position == 'final' and len(data_chunk) < slice_size:
                    data_chunk += bytes([0] * (slice_size - len(data_chunk)))

                _gh.build_spi_frame(data_chunk, frame_position)
                _gh.toggle_spi()
                time.sleep(0.05)

        except Exception as e:
            # optionally log the error
            pass
        finally:
            _gh.cleanup_spi()


    def Open_Ximea_Connection(_xi):
        _xi.cam = xiapi.Camera()
        _xi.cam.open_device()
        # print("Ximea device opened")
        _xi.Set_Ximea_Parameters()
        _xi.cam.start_acquisition()
        _xi.ximea_status = True
        # print("Ximea acquisition started")

    def Capture_Ximea_Image(_cx):
        """Capture a single frame from the Ximea camera."""
        try:
            _cx.img = xiapi.Image()
            _cx.cam.get_image(_cx.img)
            np_data = _cx.img.get_image_data_numpy()
            _cx.ximea_sensor_temp_value = _cx.cam.get_sensor_board_temp()
            _cx.hyperspectral_frames.append(np_data)
            _cx.num_frames_captured += 1
            _cx.active_mode_telem = 0x30
            return np_data
        except Exception as e:
            # print(f"Error capturing Ximea image: {e}")
            return None
        finally:
            _cx.ximea_sensor_temp_value = 100


    def Ximea_Add_Crosshair(_ch, image):
        height, width = image.shape
        center_x, center_y = width // 2, height // 2
        color = 255
        thickness = 1
        cv2.line(image, (0, center_y), (width, center_y), color, thickness)
        cv2.line(image, (center_x, 0), (center_x, height), color, thickness)
        return image

    def Close_Ximea_Connection(_cx):
        """Close the connection to the Ximea camera."""
        try:
            _cx.cam.stop_acquisition()
            _cx.cam.close_device()
            _cx.ximea_status = False
            # print("Ximea device closed")
        except Exception as e:
            # print(f"Error closing Ximea connection: {e}")
            pass

    def Set_Ximea_Parameters(_xp):
        """Set parameters for the Ximea camera."""
        try:
            _xp.cam.set_exposure(_xp.ximea_exp_time)
            _xp.cam.set_gain(_xp.ximea_gain)
            _xp.cam.set_offsetX(_xp.ximea_prm_offset_x)
            _xp.cam.set_offsetY(_xp.ximea_prm_offset_y)
            _xp.cam.set_width(_xp.ximea_xi_prm_width)
            _xp.cam.set_height(_xp.ximea_xi_prm_height)
            match _xp.ximea_img_format:
                case 0:
                    _xp.cam.set_param('imgdataformat', 'XI_MONO8')
                case 1:
                    _xp.cam.set_param('imgdataformat', 'XI_MONO16')
                case 2:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB24')
                case 3:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB32')
                case 4:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB_PLANAR')
                case 5:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW8')
                case 6:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW16')
                case 7:
                    _xp.cam.set_param('imgdataformat', 'XI_FRM_TRANSPORT_DATA')
                case 8:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB48')
                case 9:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB64')
                case 10:
                    _xp.cam.set_param('imgdataformat', 'XI_RGB16_PLANAR')
                case 11:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW8X2')
                case 12:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW8X4')
                case 13:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW16X2')
                case 14:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW16X4')
                case 15:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW32')
                case 16:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW32FLOAT')
                case 17:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW8X3')
                case 18:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW16X3')
                case _:
                    _xp.cam.set_param('imgdataformat', 'XI_RAW16')
            match _xp.ximea_trigger_src:
                case 0:
                    _xp.cam.set_trigger_source("XI_TRG_OFF")
                case 1:
                    _xp.cam.set_trigger_source("XI_TRG_SOFTWARE")
                case 2:
                    _xp.cam.set_trigger_source("XI_TRG_EDGE_RISING")
                case 3:
                    _xp.cam.set_trigger_source("XI_TRG_EDGE_FALLING")
                case 4:
                    _xp.cam.set_trigger_source("XI_TRG_LEVEL_HIGH")
                case 5:
                    _xp.cam.set_trigger_source("XI_TRG_LEVEL_LOW")
                case _:
                    _xp.cam.set_trigger_source("XI_TRG_OFF")
            _xp.cam.set_acq_timing_mode("XI_ACQ_TIMING_MODE_FRAME_RATE")
            _xp.cam.set_framerate(_xp.ximea_target_fps)
            # print("Ximea parameters set")
        except Exception as e:
            # print(f"Error setting Ximea parameters: {e}")
            pass

    def Save_Hyperspectral_Data(_sh):
        """
        Stacks captured hyperspectral frames into a cube, saves the raw data
        in a new subfolder (one per image index) and returns the raw file's full path.
        """
        base_dir = _sh.hyperspectral_data_dir
        try:
            if _sh.hyperspectral_frames:
                os.makedirs(base_dir, exist_ok=True)
                # Look for existing subfolders matching "hyperspectral_<number>"
                existing_dirs = [
                    d for d in os.listdir(base_dir)
                    if os.path.isdir(os.path.join(base_dir, d)) and re.match(r"hyperspectral_\d+", d)
                ]
                # Determine the next image index
                file_index = 1
                if existing_dirs:
                    indexes = [int(re.search(r"hyperspectral_(\d+)", d).group(1))
                            for d in existing_dirs]
                    file_index = max(indexes) + 1

                # Create a subfolder for this image index
                folder_name = f"hyperspectral_{file_index}"
                folder_path = os.path.join(base_dir, folder_name)
                os.makedirs(folder_path, exist_ok=True)

                # Save raw file inside this folder
                raw_filename = os.path.join(folder_path, f"hyperspectral_{file_index}.h5")
                hyperspectral_cube = np.dstack(_sh.hyperspectral_frames)
                with h5py.File(raw_filename, 'w') as hf:
                    hf.create_dataset('dataset', data=hyperspectral_cube)
                _sh.get_logger().info(f"Saved raw hyperspectral data to {raw_filename}")
                return raw_filename
            else:
                _sh.get_logger().error("No hyperspectral frames captured.")
                return None
        except Exception as e:
            _sh.get_logger().error(f"Error saving hyperspectral data: {e}")
            return None
        finally:
            _sh.get_last_hs_file_info()

    def compress_file_bz2(self, input_file):
        """
        Reads the 'dataset' from the input HDF5 file and writes it into a new HDF5 file
        using BZip2 compression. The output file will have the same name as the raw file,
        with '_bz2' appended before the .h5 extension.
        """
        output_file = input_file.replace(".h5", "_bz2.h5")
        try:
            start_time = time.time()
            # Instead of using a dictionary with key 'filters', get the filter parameters directly.
            compression_filter = hdf5plugin.BZip2()  # returns a dict of keyword arguments
            # Open the input file and read the dataset and its attributes.
            with h5py.File(input_file, 'r') as src:
                data = src['dataset'][:]  # load the entire dataset into memory
                attributes = dict(src['dataset'].attrs)
            
            # Create the output file and write the dataset with compression.
            with h5py.File(output_file, 'w') as dst:
                # Pass the filter parameters directly
                ds = dst.create_dataset('dataset', data=data, **compression_filter)
                # Copy all attributes.
                for key, value in attributes.items():
                    ds.attrs[key] = value

            elapsed = time.time() - start_time
            self.get_logger().info(f"[BZIP2] Compression complete in {elapsed:.2f}s")
            orig_size = os.path.getsize(input_file)
            comp_size = os.path.getsize(output_file)
            ratio = (orig_size / comp_size) if comp_size else float('inf')
            self.get_logger().info(
                f"Raw: {orig_size/(1024*1024):.2f} MB, Compressed: {comp_size/(1024*1024):.2f} MB, Ratio: {ratio:.2f}x"
            )
        except Exception as e:
            self.get_logger().error(f"BZIP2 Compression failed: {e}")
        finally:
            self.get_last_hs_file_info()


    def run_ai(_ra):
        pass

    def set_target_temp(_st, target_temp):
        target_temp_msg = TargetTemp()
        def process_temp(value):
            if value is not None:
                return int(abs(value)), value >= 0
            else:
                return 0, True
        target_temp_msg.target_temp_value, target_temp_msg.target_temp_sign = process_temp(target_temp)
        _st.target_temp_publisher.publish(target_temp_msg)

    def lucm_mode(_lm, mode, vf_frequency):
        """
        Updates and saves the LUCM mode to a CSV file, creating the file and directory
        if they do not exist. Also applies the newly set mode.
        """
        try:
            # Make sure the directory for the CSV exists
            os.makedirs(_lm.mode_directory, exist_ok=True)

            # For example, if we need the frequency to be a multiple of 15:
            if vf_frequency % 15 != 0:
                # _lm.get_logger().warning(
                #     f"VF frequency {vf_frequency} is not a multiple of 15. "
                #     f"Defaulting to 900."
                # )
                vf_frequency = 15

            # Write CSV in decimal form (mode is an integer, e.g. 16 means 0x10)
            with open(_lm.mode_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([f"0x{mode:X}"])  # e.g. "0x10"
                writer.writerow([vf_frequency])


            # Update class attributes
            _lm.vf_frequency_value = vf_frequency
            _lm.saved_mode = mode

            # Log the change in both decimal and hex form
            # _lm.get_logger().info(
            #     f"Mode changed to {mode} (hex: {hex(mode)}) with VF frequency {vf_frequency}, "
            #     f"saved to {_lm.mode_file}."
            # )

            # Apply the mode change
            _lm.mode_handler(mode)

        except Exception as e:
            pass
            # _lm.get_logger().error(f"Mode change failed: {e}")



    def compress_img(_ci):
        pass

    def factory_reset(_fr):
        pass

    def run_scenario(_rs):
        pass

    def default_callback(_cb):
        pass

    def get_custom_time_callback(_cb, msg):
        _cb.custom_time["seconds"] = msg.seconds
        _cb.custom_time["milliseconds"] = msg.milliseconds

    def get_date_from_timestamp(_gd, custom_time):
        _gd.total_seconds = custom_time["seconds"] + custom_time["milliseconds"] / 1000
        _gd.date_time = datetime.fromtimestamp(_gd.total_seconds)
        return _gd.date_time.strftime("%Y-%m-%d %H:%M:%S.%f")

    def calc_checksum(_cs, data):
        checksum = 0
        for byte in data:
            checksum += byte
            # checksum &= 0xFFFF

        lower_byte = checksum & 0xFF
        upper_byte = (checksum >> 8) & 0xFF

        checksum &= 0xFFFF

        return lower_byte, upper_byte

    def build_spi_frame(_bs, data_chunk, frame_position='middle'):
        try:
            spi_packetHEADER_1 = 0xE2
            spi_packetHEADER_2 = 0x25
            spi_version_number_1 = 0x08
            spi_version_number_2 = 0x9D

            # Determine the grouping flag based on the frame's position in the sequence
            if frame_position == 'first':
                spi_grouping_flag = 0b01  # 01 for the first frame
            elif frame_position == 'final':
                spi_grouping_flag = 0b10  # 10 for the final frame
            else:
                spi_grouping_flag = 0b00  # 00 for frames in the middle

            # Increment the packet_sequence_count each time the frame is built
            if not hasattr(_bs, 'packet_sequence_count'):
                _bs.packet_sequence_count = 0
            else:
                _bs.packet_sequence_count += 1
            combined_value = (spi_grouping_flag << 14) | (_bs.packet_sequence_count & 0x3FFF)
            combined_bytes = combined_value.to_bytes(2, byteorder='big')

            _bs.spi_frame = [
                spi_packetHEADER_1,
                spi_packetHEADER_2,
                spi_version_number_1,
                spi_version_number_2
            ]
            _bs.spi_frame.extend(combined_bytes)
            spi_packetLength_1 = 0x08
            spi_packetLength_2 = 0x07
            _bs.spi_frame.extend([
                spi_packetLength_1,
                spi_packetLength_2
            ])
            spi_seconds = _bs.custom_time['seconds'].to_bytes(4, 'big')
            spi_milli = _bs.custom_time['milliseconds'].to_bytes(2, 'big')
            _bs.spi_frame.extend(spi_seconds)
            _bs.spi_frame.extend(spi_milli)
            # Append the data chunk to the SPI frame
            _bs.spi_frame.extend(data_chunk)
            
            # Calculate remaining length and pad if necessary
            remaining_length = 2062 - len(_bs.spi_frame)
            if remaining_length > 0:
                padding_length = remaining_length
                data_chunk_padding = bytes([0] * padding_length)
                _bs.spi_frame.extend(data_chunk_padding)
            
            # Calculate checksum once and append
            lower_byte, upper_byte = _bs.calc_checksum(_bs.spi_frame[8:2062])
            _bs.spi_frame.append(upper_byte)
            _bs.spi_frame.append(lower_byte)
            _bs.print_spi_frame_in_hex()
        except Exception as e:
            pass

    def print_spi_frame_in_hex(_bs):
        """
        Stores the hexadecimal values of the SPI frame into a CSV file in a single row, with spaces between each byte,
        and without the '0x' prefix.
        :param _bs: The object containing the SPI frame and logger.
        """
        save_dir = _bs.vf_images_dir
        csv_filename = os.path.join(save_dir, f"view_{_bs.extracted_vf_image_index}.csv")

        if not _bs.spi_frame:
            # _bs.get_logger().info("SPI frame is empty. Nothing to save.")
            return

        # Convert each element of the SPI frame to hexadecimal format without the '0x' prefix
        hex_elements = [f'{byte:02X}' for byte in _bs.spi_frame]

        # Join all hex values with a space
        hex_string = ' '.join(hex_elements)
        try:
            # Open the CSV file in append mode
            with open(csv_filename, mode='a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([hex_string])
            
            _bs.get_logger().info(f"SPI frame data saved to {csv_filename}")
        except Exception as e:
            pass


    # def print_spi_data_bytes_in_hex(_bs,data_Chunk):
    #     """
    #     Stores the hexadecimal values of the SPI frame into a CSV file in a single row, with spaces between each byte,
    #     and without the '0x' prefix.

    #     :param _bs: The object containing the SPI frame and logger.
    #     """
    #     save_dir = "/home/egsa/vf_images"
    #     csv_filename = os.path.join(save_dir, f"view_data_{_bs.extracted_vf_image_index}.csv")

    #     if not _bs.spi_frame:
    #         _bs.log("err", "SPI frame is empty. Nothing to save.")
    #         return

    #     # Convert each element of the SPI frame to hexadecimal format without the '0x' prefix
    #     hex_elements = [f'{byte:02X}' for byte in data_Chunk]

    #     # Join all hex values with a space
    #     hex_string = ' '.join(hex_elements)
    #     try:
    #         # Open the CSV file in append mode
    #         with open(csv_filename, mode='a', newline='') as csv_file:
    #             csv_writer = csv.writer(csv_file)
    #             csv_writer.writerow([hex_string])
    #         _bs.log("info", f"SPI frame data saved to {csv_filename}")
    #     except Exception as e:
    #         _bs.log("err", f"Error saving SPI frame to CSV: {e}")


    def spi_config(_sc):
        # Initialize SPI communication
        _sc.cleanup_spi()
        _sc.spi = spidev.SpiDev()
        _sc.spi.open(_sc.spi_bus, _sc.spi_device)
        _sc.spi.max_speed_hz = 1000000
        _sc.spi.mode = 0b00

    def toggle_spi(_ts):
        with _ts.spi_lock:  # Ensure exclusive access to SPI
            if _ts.active_mode not in [0x20, 0x40]:
                # _ts.get_logger().warning(f"toggle_spi: Invalid mode {hex(_ts.active_mode)}")
                _ts.cleanup_spi()
                return
            try:
                _ts.spi_config()
                # _ts.get_logger().info("SPI configured, sending frame...")
                # Send the SPI frame
                # print("SPI Frame:", _ts.spi_frame)
                _ts.spi.xfer2(_ts.spi_frame)
                # _ts.get_logger().info(f"SPI sent {len(_ts.spi_frame)}")
            except Exception as e:
                pass
                # _ts.get_logger().error(f"SPI error: {str(e)}")
            finally:
                _ts.cleanup_spi()

    def get_vf_img(_gi, vf_image_index):
        """
        Reads a raw VF image file (assumed to be uncompressed RGB data), converts it
        into a NumPy array with dimensions (height, width, channels), flattens the array,
        and transmits the pixel data over SPI in 2048-byte chunks.
        """
        _gi.packet_sequence_count = 0        
        try:
            # Step 1: Locate the saved bmp image file
            save_dir = _gi.vf_images_dir  # Directory where images are saved
            bmp_image_path = os.path.join(save_dir, f"view_{vf_image_index}.bmp")

            if not os.path.exists(bmp_image_path):
                # print(f"Error: Image file view_{vf_image_index}.bmp not found in {save_dir}")
                return

            # Step 2: Read the bmp file as binary data
            with open(bmp_image_path, "rb") as f:
                binary_data = f.read()

            # Step 3: Slice the binary data into chunks of 2048 bytes
            chunk_size = 2048
            total_bytes = len(binary_data)
            num_chunks = (total_bytes + chunk_size - 1) // chunk_size  # Round up

            # Step 4: Loop through the chunks and send each as an SPI frame
            for i in range(num_chunks):
                if _gi.abort_event.is_set():
                    break
                frame_position = 'middle'
                start_idx = i * chunk_size
                end_idx = start_idx + chunk_size

                if i == 0:
                    frame_position = 'first'
                elif i == num_chunks - 1:
                    frame_position = 'final'

                # Extract the current chunk
                data_chunk = binary_data[start_idx:end_idx]

                # If this is the final frame and the data chunk is less than 2048 bytes, pad with zeros
                if frame_position == 'final' and len(data_chunk) < chunk_size:
                    padding_length = chunk_size - len(data_chunk)
                    data_chunk += bytes([0] * padding_length)

                # Build the SPI frame for the current chunk
                _gi.build_spi_frame(data_chunk, frame_position)

                # Send the SPI frame
                _gi.toggle_spi()
                time.sleep(0.05)

            # print(f"Transmission of bmp image {vf_image_index} complete.")

        except Exception as e:
            # print(f"Exception in get_vf_img: {e}")
            pass
        finally:
            # Clean up resources
            _gi.cleanup_spi()
            _gi.packet_sequence_count = 0        


    def delete_hs_img(_di, delete_flag, image_index):
        """
        Deletes HS imagery according to the delete_flag:
        1 => only the raw .h5 file
        2 => only the compressed _bz2.h5 file
        5 => the entire folder hyperspectral_<index>
        """
        folder_name = f"hyperspectral_{image_index}"
        folder_path = os.path.join(_di.hyperspectral_data_dir, folder_name)
        try:
            if delete_flag == 1:
                raw_file = os.path.join(folder_path, f"hyperspectral_{image_index}.h5")
                if os.path.exists(raw_file):
                    os.remove(raw_file)
            elif delete_flag == 2:
                compressed_file = os.path.join(folder_path, f"hyperspectral_{image_index}_bz2.h5")
                if os.path.exists(compressed_file):
                    os.remove(compressed_file)
            elif delete_flag == 5:
                if os.path.exists(folder_path):
                    shutil.rmtree(folder_path)
        except Exception as e:
            pass
        finally:
            _di.get_last_hs_file_info()


    def delete_vf_img(_di, delete_flag, image_index):
        try:
            if delete_flag == 1:
                # Step 1: Locate the saved image file
                save_dir = _di.vf_images_dir  # Directory where images are saved
                image_path = os.path.join(save_dir, f"view_{image_index}.bmp")
                if not os.path.exists(image_path):
                    # print(f"Image file hyperspectral_{image_index}.h5 not found in {save_dir}")
                    return
                _di.deleted_img = os.remove(image_path)
        except Exception as e:
            pass
        finally:
            _di.get_last_vf_file_info()

    def publish_payload_telem(_pt):
        pld_telem_msg = PldTelem()
        def process_temp(value):
            if value is not None:
                return int(abs(value) * 100), value >= 0
            else:
                return 0, True
        pld_telem_msg.mode = _pt.active_mode_telem
        pld_telem_msg.pld_cmd_counter = _pt.cmd_counter
        pld_telem_msg.pld_excuted_cmd_counter = _pt.exc_cmd_counter
        pld_telem_msg.vf_img_index = _pt.extracted_vf_image_index
        pld_telem_msg.vf_img_size = _pt.vf_image_size
        pld_telem_msg.hs_img_index = _pt.extracted_hs_image_index
        pld_telem_msg.compressed_hs_img_size = _pt.hs_compressed_image_size
        pld_telem_msg.ximea_temp_value, pld_telem_msg.ximea_temp_sign = process_temp(_pt.ximea_sensor_temp_value)
        pld_telem_msg.hs_img_size = _pt.hs_image_size
        pld_telem_msg.hs_cam_status = _pt.ximea_status
        pld_telem_msg.vf_cam_status = _pt.vf_cam_status
        pld_telem_msg.last_excuted_command = _pt.last_exc_cmd
        pld_telem_msg.payload_id_err = _pt.payload_id_error
        _pt.payload_telem_publisher.publish(pld_telem_msg)
        # _pt.get_logger().info(_pt.state)

def main(args=None):
    rclpy.init(args=args)
    payload = Payload()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(payload)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        payload.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
