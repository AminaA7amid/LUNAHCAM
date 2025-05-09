"""
-   A python Helper file that Handles Command Interface responsibilities:
    -   Frame Packetization/Depacketization
    -   Frame integrity/Assertion checks (refer to Chinese document)
    -   Checksum calculation
"""

import os
import sys
import subprocess
import time
from rclpy.logging import get_logger

# directory mapping for importing local files
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from protocol_config import ProtocolConfig
from log_level import LogLevel

class Protocol(ProtocolConfig):
    def __init__(_p):
        ProtocolConfig.__init__(_p)
        _p.logger = get_logger('ProtocolLogger')  # Separate logger (not a ROS2 Node)
        
        # Set default error code (no error)
        _p.errCODE = _p.errNONE  
        
        # Set default response code (normal instruction)
        _p.resCODE = _p.normalINSTRUCTION  
        
        # Updated error mapping – only the seven errors (plus no_err)
        _p.errors = {
            "0":   "NO_ERR",
            "17":  "PARITY_ERR",
            "18":  "STOP_BIT_ERR",
            "19":  "CHAR_SPACE_ERR",
            "33":  "PAYLOAD_ID_ERR",
            "34":  "INSTRUCTION_ERR",
            "35":  "CMD_LEN_ERR",
            "36":  "CHECKSUM_ERR"
        }
        
        # Data length checks for each command (as before)
        _p.data_len_check = {
            str(_p.cmdTIME_SYNC): 6,
            str(_p.cmdTELEM_POLLING): 2,
            str(_p.cmdDATA_INJECTION): 57,
            str(_p.cmdPRE_SHUTDOWN): 2,
        }
        
        # Fixed data content for some commands:
        _p.data_check = {
            str(_p.cmdTELEM_POLLING): [0x5A, 0x5A],
            str(_p.cmdPRE_SHUTDOWN): [0x94, 0x94],
        }
        
        # Initial empty frame template for responses:
        _p.FRAME = [_p.packetHEADER_1,
                    _p.packetHEADER_2,
                    0,  # placeholder: payload_id
                    0]  # placeholder: response_code
        
        # Allowed destination (our payload id is 0x06)
        _p.allowed_DEST = [6]
        _p.analyze_frame_duration = 0 
        _p.packetize_duration = 0
        _p.depacketize_duration = 0
        _p.calcualte_checksum_duration = 0

        # Allowed command IDs:
        _p.allowed_CMDS = [
            _p.cmdTIME_SYNC,
            _p.cmdTELEM_POLLING,
            _p.cmdDATA_INJECTION,
            _p.cmdPRE_SHUTDOWN,
        ]
        
    def calc_checksum(self, data):
        start_time = time.perf_counter()  # Record start time
        checksum = 0
        for byte in data:
            checksum += byte
        lower_byte = checksum & 0xFF
        upper_byte = (checksum >> 8) & 0xFF
        checksum &= 0xFFFF
        end_time = time.perf_counter()  # Record end time
        self.calcualte_checksum_duration = (end_time - start_time) * 1000  # Convert to ms
        return lower_byte, upper_byte


    def analyze_frame(_chk, frame):
        start_time = time.perf_counter()  # Record start time
        try:
            if all(byte == 0 for byte in frame):
                return  # Do not process an all-zero frame
            checksum_lower_byte, checksum_higher_byte = _chk.calc_checksum(
                frame[_chk.idxPAYLOAD_ID : _chk.idxCHECKSUM_HIGHER_BYTE]
            )
            assert frame[_chk.idxPACKETHEADER_1] == _chk.packetHEADER_1 and \
                frame[_chk.idxPACKETHEADER_2] == _chk.packetHEADER_2, 34
            assert frame[_chk.idxCHECKSUM_HIGHER_BYTE] == checksum_higher_byte and \
                frame[_chk.idxCHECKSUM_LOWER_BYTE] == checksum_lower_byte, 36
            assert frame[_chk.idxCMD_ID] in _chk.allowed_CMDS, 34
            assert frame[_chk.idxPAYLOAD_ID] in _chk.allowed_DEST, 33
            expected_len = _chk.data_len_check[str(frame[_chk.idxCMD_ID])]
            actual_len = frame[_chk.idxDATA_LEN] + 1
            assert actual_len == expected_len, 35
            assert actual_len == len(frame[_chk.idxDATA_START : _chk.idxCHECKSUM_HIGHER_BYTE]), 35
            if frame[_chk.idxCMD_ID] in [ _chk.cmdTELEM_POLLING, _chk.cmdPRE_SHUTDOWN ]:
                assert list(frame[_chk.idxDATA_START : _chk.idxCHECKSUM_HIGHER_BYTE]) == _chk.data_check[str(frame[_chk.idxCMD_ID])], 34
            _chk.errCODE = _chk.errNONE
            _chk.resCODE = _chk.normalINSTRUCTION
        except AssertionError as e:
            _chk.errCODE = e.args[0] if e.args and isinstance(e.args[0], int) else 34
            _chk.resCODE = _chk.errorINSTRUCTION
        end_time = time.perf_counter()  # Record end time
        _chk.analyze_frame_duration = (end_time - start_time) * 1000  # Convert to ms
        # _chk.logger.info(f"analyze_frame execution time: {duration:.2f} ms")
        return _chk.errCODE, _chk.resCODE

    def packetize(_pkt, res_code, payload=None):
        start_time = time.perf_counter()
        _pkt.FRAME = []
        if not payload:
            _pkt.FRAME.extend([_pkt.packetHEADER_1, _pkt.packetHEADER_2])
            _pkt.FRAME.extend([_pkt.allowed_DEST[0]])
            _pkt.FRAME.extend([res_code])
        else:
            checksum_lower_byte, checksum_higher_byte = _pkt.calc_checksum(payload[6:])
            _pkt.FRAME.extend([_pkt.packetHEADER_1, _pkt.packetHEADER_2])
            _pkt.FRAME.extend(payload)
            _pkt.FRAME.extend([checksum_higher_byte, checksum_lower_byte])

        end_time = time.perf_counter()
        _pkt.packetize_duration = (end_time - start_time) * 1000
        # _pkt.logger.info(f"packetize execution time: {duration:.2f} ms")
        return _pkt.FRAME

    def depacketize(_dpkt, frame):
        start_time = time.perf_counter()
        analyze_result = _dpkt.analyze_frame(frame)
        if analyze_result is not None:
            err_code, res_code = analyze_result
            if err_code != _dpkt.errNONE:
                return {
                    "err_code": _dpkt.errors[str(err_code)],
                    "res_code": res_code
                }
        data_len = frame[_dpkt.idxDATA_LEN] + 1
        response = {
            "res_code": _dpkt.resCODE,
            "header_1": frame[_dpkt.idxPACKETHEADER_1],
            "header_2": frame[_dpkt.idxPACKETHEADER_2],
            "payload_id": frame[_dpkt.idxPAYLOAD_ID],
            "cmd": frame[_dpkt.idxCMD_ID],
            "data_len": frame[_dpkt.idxDATA_LEN],
            "data": frame[_dpkt.idxDATA_START : _dpkt.idxDATA_START + data_len],
            "checksum_lower_byte": frame[_dpkt.idxCHECKSUM_LOWER_BYTE],
            "checksum_higher_byte": frame[_dpkt.idxCHECKSUM_HIGHER_BYTE],
        }
        end_time = time.perf_counter()
        _dpkt.depacketize_duration = (end_time - start_time) * 1000
        # _dpkt.logger.info(f"depacketize execution time: {duration:.2f} ms")
        return response

    def turn_on_wifi(_on):
        on_commands = [
        "sudo modprobe brcmfmac",
        "sudo systemctl unmask systemd-networkd",
        "sudo systemctl unmask wpa_supplicant",
        "sudo systemctl unmask systemd-networkd.service",
        "sudo systemctl unmask systemd-resolved.service",
        "sudo systemctl unmask systemd-rfkill.service",
        "sudo systemctl unmask systemd-networkd.socket",
        "sudo systemctl unmask systemd-logind.service",
        "sudo systemctl unmask netplan-wpa-wlan0.service",
        "sudo systemctl start systemd-networkd",
        "sudo systemctl start wpa_supplicant",
        "sudo systemctl start systemd-networkd.service",
        "sudo systemctl start systemd-resolved.service",
        "sudo systemctl start systemd-rfkill.service",
        "sudo systemctl start systemd-networkd.socket",
        "sudo systemctl start systemd-logind.service",
        "sudo systemctl start netplan-wpa-wlan0.service",
        "sudo systemctl daemon-reload",
        "sudo netplan apply"
    ]
        for command in on_commands:
            try:
                subprocess.run(command, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                pass
            pass
    def turn_off_wifi(_off):
        off_commands = [
        "sudo systemctl stop systemd-networkd",
        "sudo systemctl stop wpa_supplicant",
        "sudo systemctl stop systemd-networkd.service",
        "sudo systemctl stop systemd-resolved.service",
        "sudo systemctl stop systemd-rfkill.service",
        "sudo systemctl stop systemd-networkd.socket",
        "sudo systemctl stop systemd-networkd-wait-online.service",
        "sudo systemctl stop systemd-logind.service",
        "sudo systemctl stop netplan-wpa-wlan0.service",
        "sudo systemctl disable systemd-networkd",
        "sudo systemctl disable wpa_supplicant",
        "sudo systemctl disable systemd-networkd.service",
        "sudo systemctl disable systemd-resolved.service",
        "sudo systemctl disable systemd-rfkill.service",
        "sudo systemctl disable systemd-networkd.socket",
        "sudo systemctl disable systemd-networkd-wait-online.service",
        "sudo systemctl disable systemd-logind.service",
        "sudo systemctl disable netplan-wpa-wlan0.service",
        "sudo systemctl mask systemd-networkd",
        "sudo systemctl mask wpa_supplicant",
        "sudo systemctl mask systemd-networkd.service",
        "sudo systemctl mask systemd-resolved.service",
        "sudo systemctl mask systemd-rfkill.service",
        "sudo systemctl mask systemd-networkd.socket",
        "sudo systemctl mask systemd-networkd-wait-online.service",
        "sudo systemctl mask systemd-logind.service",
        "sudo systemctl mask netplan-wpa-wlan0.service",
        "sudo systemctl daemon-reload"
        ]
        for command in off_commands:
            try:
                subprocess.run(command, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                pass
            pass


    def disable_usb(_off):
        off_commands = [
        "sudo uhubctl -l 1-1 -p 1 -a off",
        "sudo uhubctl -l 1-1 -p 2 -a off",
        "sudo uhubctl -l 1-1 -p 3 -a off",
        "sudo uhubctl -l 1-1 -p 4 -a off"
        ]
        for command in off_commands:
            try:
                subprocess.run(command, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                pass
            pass

    def enable_usb(_off):
        off_commands = [
        "sudo uhubctl -l 1-1 -p 1 -a on",
        "sudo uhubctl -l 1-1 -p 2 -a on",
        "sudo uhubctl -l 1-1 -p 3 -a on",
        "sudo uhubctl -l 1-1 -p 4 -a on",
        "echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null"
        ]
        for command in off_commands:
            try:
                subprocess.run(command, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                pass
            pass
    def time_logger(_tl):
        _tl.logger.info(f"analyze_frame execution time: {_tl.analyze_frame_duration:.2f} ms")
        _tl.logger.info(f"Depacketize execution time: {_tl.depacketize_duration:.2f} ms")
        _tl.logger.info(f"Depacketize execution time: {_tl.packetize_duration:.2f} ms")
        _tl.logger.info(f"Calculate Checksum Execution time: {_tl.calcualte_checksum_duration:.2f} ms")
    # def pre_shutdown_callback(self):
    #     try:
    #         subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
    #         raise SystemExit
    #     except Exception as e:
    #         # print(f'Error: {e}')
    #         pass

