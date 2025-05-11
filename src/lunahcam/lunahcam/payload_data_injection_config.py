"""
-   A python configuration files that loads configuration parameters from payload_data_injection_config.json
-   Configuration is for the Data injection command capture img, get_img ...etc which will be wrapped in the data injection command
"""

import json
import os

"""
-   valid data field in the injection frame is as follows:
    - Capture img
    [
        cmd_id, (1 byte)
        payload_id (which camera), (1 byte)
        gain, (2 bytes) higher byte is integer component, lower byte is decimal component ex: [2,5] => gain is 2.5
        exp_time, (4 bytes) in microseconds
        number_of_frames, (2 bytes) number of frames to be captured
        fps, (1 byte) frame rate
        compression, (1 byte) compression or no compression (0 for no compression,1 for compression)
        time_of_capture (1 byte) time in seconds for capturing
    ]
    
    - get img
    [
        cmd_id, (1 byte)
        payload_id (which camera), (1 byte)
        img index, (1 byte)
        number of frames, (2 bytes)
    ]
"""

class PayloadConfig:
    def __init__(_cfg):

        # Load configuration data from 'payload_data_injection_config.json' file and set class attributes based on the data.
        file = open('/LUNAHCAM_V1/LUNAHCAM/src/lunahcam/config/payload_data_injection_config.json')
        payload_config = json.load(file)
        
        # Command IDs
        _cfg.cmdCAPTURE_IMAGE = payload_config["commands"]["capture_image"]
        _cfg.cmdGET_TELEMETRY = payload_config["commands"]["get_telemetry"]
        _cfg.cmdGET_IMAGE = payload_config["commands"]["get_image"]
        _cfg.cmdDELETE_IMAGE = payload_config["commands"]["delete_image"]
        _cfg.cmdRUN_AI = payload_config["commands"]["run_ai_pipeline"]
        _cfg.cmdSET_TEMP = payload_config["commands"]["set_target_temp"]
        _cfg.cmdLUCM_MODE = payload_config["commands"]["set_lucm_mode"]
        _cfg.cmdIMG_COMP = payload_config["commands"]["set_img_comp"]
        _cfg.cmdFACTORY_RESET = payload_config["commands"]["factory_reset"]
        _cfg.cmdRUN_SCENARIO = payload_config["commands"]["run_scenario"]
        _cfg.cmdTURN_0N_WIFI = payload_config["commands"]["turn_on_wifi"]
        _cfg.cmdTURN_OFF_WIFI = payload_config["commands"]["turn_off_wifi"]

        # Payload IDs
        _cfg.ximea_ID = payload_config["payload_ids"]["ximea"]
        _cfg.rpiCAM_ID = payload_config["payload_ids"]["rpi_cam"]

        # Data indices for Capture Image Command
        _cfg.idxCMD_ID = payload_config["capture_image"]["indices"]["cmd_id"]
        _cfg.idxPAYLOAD_ID = payload_config["capture_image"]["indices"]["payload_id"]
        _cfg.idxGAIN = payload_config["capture_image"]["indices"]["gain"]
        _cfg.idxEXP_TIME = payload_config["capture_image"]["indices"]["exp_time"]
        _cfg.idxNUMBER_OF_FRAMES = payload_config["capture_image"]["indices"]["number_of_frames"]
        _cfg.idxFPS = payload_config["capture_image"]["indices"]["fps"]
        _cfg.idxCOMPRESSION = payload_config["capture_image"]["indices"]["compression"]
        _cfg.idxTIME_OF_CAPTURE = payload_config["capture_image"]["indices"]["time_of_capture"]
        
        # Data indices for Get Image Command
        _cfg.idxCMD_ID = payload_config["get_image"]["indices"]["cmd_id"]
        _cfg.idxPAYLOAD_ID = payload_config["get_image"]["indices"]["payload_id"]
        _cfg.idxIMG_INDEX = payload_config["get_image"]["indices"]["img_index"]
        _cfg.idxGET_NUMBER_OF_FRAMES = payload_config["get_image"]["indices"]["number_of_frames"]

        # Data indices for Delete Image Command
        _cfg.idxCMD_ID = payload_config["delete_image"]["indices"]["cmd_id"]
        _cfg.idxPAYLOAD_ID = payload_config["delete_image"]["indices"]["payload_id"]
        _cfg.idxIMG_INDEX = payload_config["delete_image"]["indices"]["img_index"]
        _cfg.idxDELETE_NUMBER_OF_FRAMES = payload_config["delete_image"]["indices"]["number_of_frames"]
        _cfg.idxDELETE_FLAG = payload_config["delete_image"]["indices"]["delete_flag"]