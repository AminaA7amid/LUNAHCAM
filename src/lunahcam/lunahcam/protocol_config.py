"""
-   A python configuration files that loads configuration parameters from protocol_config.json
-   Connfiguration is for the commands,replies,packet structure ...etc
"""
import json
import os

"""
-   The command interface is an RS422 protocol that uses UART on the raspberry pi with the following configuration:
    1 start bit
    2 stop bit
    odd parity
    8 - bit data
    115200 baudrate
-   command frame is as follows:
    [
        packet_header_1 : fixed as 0xEB (1 byte)
        packet_header_2 : fixed as 0x90 (1 byte)
        payload_id: fixed as 0x06 (1 byte)
        cmd_id: (refer to the chineese interface document) (1 byte)
        data_len: (actual data length - 1) ex. if data is 5 bytes, this field will be 0x04
        data: valid data to be sent (length varies according to the command, refer to the chineese document)
        checksum higher byte: (1 byte)
        checksum lower byte: (1 byte)
    ]
-   checksum is the sum of all elements in the frame from payload id till the end of the data
    and the last 2 bytes of the results are taken as higher and lower bytes
"""
class ProtocolConfig:
    def __init__(_cfg):
        
        # Load configuration data from 'config.json' file and set class attributes based on the data.
        file = open('/home/egsa/LUNAHCAM/src/lunahcam/config/protocol_config.json')
        protocol_config = json.load(file)

        # Frame Constants
        _cfg.packetHEADER_1 = protocol_config["packet_header_1"]
        _cfg.packetHEADER_2 = protocol_config["packet_header_2"]
        _cfg.maxDATA_SIZE = protocol_config["max_data_size"]
        _cfg.minDATA_SIZE = protocol_config["min_data_size"]

        # Frame indicies
        _cfg.idxPACKETHEADER_1 = protocol_config["frame_indicies"]["packet_header_1"]
        _cfg.idxPACKETHEADER_2 = protocol_config["frame_indicies"]["packet_header_2"]
        _cfg.idxPAYLOAD_ID= protocol_config["frame_indicies"]["payload_id"]
        _cfg.idxCMD_ID = protocol_config["frame_indicies"]["cmd_id"]
        _cfg.idxDATA_LEN = protocol_config["frame_indicies"]["data_len"]
        _cfg.idxDATA_START = protocol_config["frame_indicies"]["data_start"]
        _cfg.idxCHECKSUM_HIGHER_BYTE = protocol_config["frame_indicies"]["checsum_higher_byte"]
        _cfg.idxCHECKSUM_LOWER_BYTE = protocol_config["frame_indicies"]["checsum_lower_byte"]

        # SPI Params
        _cfg.spi_packetHEADER_1 = protocol_config["spi_params"]["synchronization_code"]["packet_header_1"]
        _cfg.spi_packetHEADER_2 = protocol_config["spi_params"]["synchronization_code"]["packet_header_2"]
        _cfg.spi_version_number = protocol_config["spi_params"]["primary_header"]["version"]["version_number"]
        _cfg.spi_type = protocol_config["spi_params"]["primary_header"]["version"]["type"]
        _cfg.spi_secondary_header_flag = protocol_config["spi_params"]["packet_identifier"]["secondary_header_flag"]
        _cfg.spi_application_process_identifier = protocol_config["spi_params"]["packet_identifier"]["application_process_identifier"]
        _cfg.spi_grouping_flag = protocol_config["spi_params"]["packet_sequence_control"]["grouping_flag"]
        _cfg.packet_sequence_count = protocol_config["spi_params"]["packet_sequence_control"]["packet_sequence_count"]
        
        # Commands
        _cfg.cmdTIME_SYNC = protocol_config["commands"]["time_synchronization_command"]["id"]
        _cfg.cmdTELEM_POLLING = protocol_config["commands"]["telemetry_polling_command"]["id"]
        _cfg.cmdDATA_INJECTION = protocol_config["commands"]["data_injection_command"]["id"]
        _cfg.cmdPRE_SHUTDOWN = protocol_config["commands"]["pre_shutdown_command"]["id"]
        
        
        # Replies
        _cfg.normalINSTRUCTION = protocol_config["replies"]["normal_instruction"]
        _cfg.errorINSTRUCTION = protocol_config["replies"]["instruction_err"]
        
        # Errors (errCODE)
        _cfg.errNONE = protocol_config["errors"]["no_err"]
        _cfg.errPARITY = protocol_config["errors"]["parity_err"]
        _cfg.errSTOP_BIT = protocol_config["errors"]["stop_bit_error"]
        _cfg.errCHAR_SPACE = protocol_config["errors"]["char_space_error"]
        _cfg.errPAYLOAD_ID = protocol_config["errors"]["payload_id_error"]
        _cfg.errCMD = protocol_config["errors"]["instruction_err"]
        _cfg.errDATA_LEN = protocol_config["errors"]["cmd_len_err"]
        _cfg.errCHECKSUM = protocol_config["errors"]["checksum_err"]
        
        _cfg.errs = list(protocol_config["errors"].values())[1:]