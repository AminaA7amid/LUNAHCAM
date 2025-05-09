import rclpy
from rclpy.node import Node
import sys
import os
import time
from pl_interface.srv import BusReply
from pl_interface.msg import PldCmd, CustTime, Temp, Throttled, TelemData, PldTelem,Error
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from protocol import Protocol
from log_level import LogLevel

class CmdHandler(Node, Protocol, LogLevel):
    def __init__(_cmd):
        super().__init__('cmd_handler')
        Protocol.__init__(_cmd)
        LogLevel.__init__(_cmd)

        _cmd._allow_undeclared_parameters = True
        
        _cmd.namespace = _cmd.get_namespace()
        _cmd.ns = _cmd.namespace[1:]
        _cmd.declare_parameters(
            namespace=_cmd.namespace,
            parameters=[
                ('services.cmd_srv', ""),
                ('topics.payload', ""),
                ('topics.custom_time', ""),
                ('topics.shutdown', ""),
                ('topics.temp', "/temp"),
                ('topics.throttled', "/throttled"),
                ('topics.telemetry_data', "/telem"),
                ('topics.payload_telem','/payload_telem'),
                ('topics.error', "/error")
            ])
        _cmd.cmd_service_name = _cmd.get_parameter(f'{_cmd.namespace}.services.cmd_srv').value
        _cmd.payload_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.payload').value
        _cmd.shutdown_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.shutdown').value
        _cmd.custom_time_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.custom_time').value
        _cmd.temp_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.temp').value
        _cmd.throttled_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.throttled').value
        _cmd.telem_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.telemetry_data').value
        _cmd.payload_telem_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.payload_telem').value
        _cmd.error_topic_name = _cmd.get_parameter(f'{_cmd.namespace}.topics.error').value

        _cmd.pre_shutdown = False
        _cmd.telemetry_sequence_count = 0
        _cmd.telemetry_grouping_flag = 0b11  # or set this once if it's constant
        _cmd.telem = [
            0x00,
            0x7A,
            0x00,
            0x00,
            0x00,
            119
        ]
        _cmd.test_telem = [0] * 118
        _cmd.telem.extend(_cmd.test_telem)
        
        _cmd.custom_time = {'seconds': 0, 'milliseconds': 0}  # Initial time
        _cmd.reference_real_time = time.time()


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create services and publishers
        _cmd.cmd_handler_service = _cmd.create_service(BusReply, _cmd.cmd_service_name, _cmd.service_handler,qos_profile=qos_profile)
        _cmd.payload_command_publisher = _cmd.create_publisher(PldCmd, f'{_cmd.payload_topic_name}', 10)
        _cmd.custom_time_publisher = _cmd.create_publisher(CustTime, f'{_cmd.custom_time_topic_name}', 10)
        _cmd.pre_shutdown_publisher = _cmd.create_publisher(String, f'{_cmd.shutdown_topic_name}', 10)

        # Create subscribers
        _cmd.temp_subscriber = _cmd.create_subscription(Temp, _cmd.temp_topic_name, _cmd.update_temperature_data, 10)
        _cmd.throttled_subscriber = _cmd.create_subscription(Throttled, _cmd.throttled_topic_name, _cmd.update_throttled_status, 10)
        _cmd.telem_subscriber = _cmd.create_subscription(TelemData, _cmd.telem_topic_name, _cmd.update_telem_data, 10)
        _cmd.payload_telem_subscriber = _cmd.create_subscription(PldTelem, _cmd.payload_telem_topic_name, _cmd.update_payload_telem_data, 10)
        _cmd.error_subscriber = _cmd.create_subscription(Error, _cmd.error_topic_name, _cmd.update_error_data, 10)
        
        _cmd.payloadCMD = PldCmd()
        _cmd.custom_timeMSG = CustTime()
        _cmd.pre_shutdown_msg = String()
        _cmd.cmd_handler_cmd_counter = 0
        _cmd.pld_cmd_count = 0
        _cmd.exc_pld_cmd_count = 0

        _cmd.parity_error = 0
        _cmd.parity_error_counter = 0

        _cmd.stopbit_error = 0
        _cmd.stopbit_error_counter = 0

        _cmd.char_spacing_error = 0
        _cmd.char_spacing_error_counter = 0

        _cmd.payload_id_error = 0
        _cmd.payload_id_error_counter = 0

        _cmd.instruction_type_error = 0
        _cmd.instruction_type_error_counter = 0

        _cmd.instruction_length_error = 0
        _cmd.instruction_length_error_counter = 0
        _cmd.checksum_error = 0
        _cmd.checksum_error_counter = 0
        _cmd.error_summary = 0
        _cmd.rpi_temp = 0
        _cmd.sensor1_temp = 0
        _cmd.sensor2_temp = 0
        _cmd.sensor3_temp = 0
        _cmd.sensor4_temp = 0
        _cmd.sensor5_temp = 0
        _cmd.sensor6_temp = 0
        _cmd.sensor7_temp = 0
        _cmd.sensor8_temp = 0
        _cmd.sensor9_temp = 0
        _cmd.sensor10_temp = 0
        _cmd.sensor11_temp = 0
        _cmd.ximea_error_value = 0
        _cmd.temp_sensor_1_sign = True
        _cmd.temp_sensor_2_sign = True
        _cmd.temp_sensor_3_sign = True
        _cmd.temp_sensor_4_sign = True
        _cmd.temp_sensor_5_sign = True
        _cmd.temp_sensor_6_sign = True
        _cmd.temp_sensor_7_sign = True
        _cmd.temp_sensor_8_sign = True
        _cmd.temp_sensor_9_sign = True
        _cmd.temp_sensor_10_sign = True
        _cmd.temp_sensor_11_sign = True
        _cmd.ximea_temp_sign_updated = True
        _cmd.rpi_temp_sign = True
        _cmd.positive_sign = 0
        _cmd.negative_sign = 255
        _cmd.lucm_mode = 16
        _cmd.throttle = 0
        _cmd.cpu_percentage = 0 
        _cmd.ram_percentage = 0
        _cmd.storage_health_percentage = 0
        _cmd.vf_snap_counter = 0
        _cmd.hs_snap_counter = 0
        _cmd.hs_snap_size = 0
        _cmd.vf_snap_size = 0
        _cmd.compressed_hs_snap_size = 0
        _cmd.ximea_temp_value = 0
        _cmd.ximea_status = False
        _cmd.vf_status = False
        _cmd.heater_1_status = False
        _cmd.heater_2_status = False
        _cmd.hw_status_bits = 0x00
        _cmd.last_excuted_cmd = 0
        _cmd.callbacks = {
            str(_cmd.cmdTIME_SYNC): _cmd.time_sync_callback,
            str(_cmd.cmdTELEM_POLLING): _cmd.telem_polling_callback,
            str(_cmd.cmdDATA_INJECTION): _cmd.data_injection_callback,
            str(_cmd.cmdPRE_SHUTDOWN): _cmd.pre_shutdown_callback,
        }
        try:
            with open('/var/lib/lunahcam_boot_count.txt', 'r') as f:
                _cmd.boot_cycle_count = int(f.read().strip())
        except (FileNotFoundError, ValueError) as e:
            _cmd.boot_cycle_count = 0

    def update_error_instructions(_cmd, err_code):
        """
        Updates the error counters and sets the appropriate bit in error_summary based on the error code.
        
        Error bit mapping:
        - 17: Parity error    -> Bit 0 (0x01)
        - 18: Stop bit error  -> Bit 1 (0x02)
        - 19: Character spacing error -> Bit 2 (0x04)
        - 33: Payload ID error -> Bit 3 (0x08)
        - 34: Instruction type error -> Bit 4 (0x10)
        - 35: Instruction length error -> Bit 5 (0x20)
        - 36: Checksum error -> Bit 6 (0x40)
        """
        match err_code:
            case 17:
                _cmd.parity_error = 17
                _cmd.parity_error_counter += 1
                _cmd.error_summary |= 0x01  # Set bit 0 (0b00000001)
                if _cmd.parity_error_counter >=256:
                    _cmd.parity_error_counter = 0
            case 18:
                _cmd.stopbit_error = 18
                _cmd.stopbit_error_counter += 1
                _cmd.error_summary |= 0x02  # Set bit 1 (0b00000010)
                if _cmd.stopbit_error_counter >=256:
                    _cmd.stopbit_error_counter = 0
            case 19:
                _cmd.char_spacing_error = 19
                _cmd.char_spacing_error_counter += 1
                _cmd.error_summary |= 0x04  # Set bit 2 (0b00000100)
                if _cmd.char_spacing_error_counter>=256:
                    _cmd.char_spacing_error_counter = 0
            case 33:
                _cmd.payload_id_error = 33
                _cmd.payload_id_error_counter += 1
                _cmd.error_summary |= 0x08  # Set bit 3 (0b00001000)
                if _cmd.payload_id_error_counter>=256:
                    _cmd.payload_id_error_counter = 0
            case 34:
                _cmd.instruction_type_error = 34
                _cmd.instruction_type_error_counter += 1
                _cmd.error_summary |= 0x10  # Set bit 4 (0b00010000)
                if _cmd.instruction_type_error_counter>=256:
                    _cmd.instruction_type_error_counter = 0
            case 35:
                _cmd.instruction_length_error = 35
                _cmd.instruction_length_error_counter += 1
                _cmd.error_summary |= 0x20  # Set bit 5 (0b00100000)
                if _cmd.instruction_length_error_counter>=256:
                    _cmd.instruction_length_error_counter = 0
            case 36:
                _cmd.checksum_error = 36
                _cmd.checksum_error_counter += 1
                _cmd.error_summary |= 0x40  # Set bit 6 (0b01000000)
                if _cmd.checksum_error_counter>=256:
                    _cmd.checksum_error_counter = 0
            case _:
                pass

    def service_handler(_sh, req, res):
        if req.err:
            res.res_code = _sh.errorINSTRUCTION
            return res
        else:
            res.res_code = _sh.normalINSTRUCTION
            cmd_callback = _sh.callbacks.get(str(req.cmd), _sh.default_callback)
            return cmd_callback(req, res)

    def time_sync_callback(_cb, req, res):
        _cb.cmd_handler_cmd_counter+=1  
        time_seconds = int.from_bytes(req.data[:4], byteorder='big')
        time_milliseconds = int.from_bytes(req.data[4:], byteorder='big')
        _cb.custom_time['seconds'] = time_seconds
        _cb.custom_time['milliseconds'] = time_milliseconds
        _cb.last_excuted_cmd = 19
        return res
        
    def telem_polling_callback(_cb, req, res):
        _cb.telemetry_combined_value = (_cb.telemetry_grouping_flag << 14) | (_cb.telemetry_sequence_count & 0x3FFF)
        _cb.telemetry_combined_bytes = _cb.telemetry_combined_value.to_bytes(2, byteorder='big')
        _cb.telemetry_combined_bytes = _cb.telemetry_combined_value.to_bytes(2, byteorder='big')
        _cb.cmd_handler_cmd_counter += 1
        _cb.cmd_counter = int(_cb.cmd_handler_cmd_counter + _cb.pld_cmd_count)
        _cb.exc_pld_cmd_count = int(_cb.cmd_handler_cmd_counter)
        _cb.cmd_counter_bytes = _cb.cmd_counter.to_bytes(2, 'big')
        _cb.exc_pld_cmd_counter_bytes = _cb.exc_pld_cmd_count.to_bytes(2, 'big')
        _cb.telem[2:4] = list(_cb.telemetry_combined_bytes)
        _cb.telem[6:10] = list(_cb.custom_time['seconds'].to_bytes(4, 'big'))
        _cb.telem[10:12] = list(_cb.custom_time['milliseconds'].to_bytes(2, 'big'))
        _cb.telem[12:13] = list(_cb.lucm_mode.to_bytes(1, 'big'))  #LUCM MODE byte
        current_error = _cb.error_summary
        current_error_dupe = current_error
        _cb.telem[13:14] = list(current_error.to_bytes(1, 'big'))  #LUCM MODE byte
        _cb.error_summary = 0  # Clear error summary for subsequent frames
        _cb.telem[14:15] = [_cb.cmd_counter_bytes[1]]  # CMD Counter LSB
        _cb.telem[15:16] = [_cb.exc_pld_cmd_counter_bytes[1]]  # EXC CMD Counter LSB
        _cb.telem[16:17] = list(_cb.last_excuted_cmd.to_bytes(1, 'big')) #LUCM Excuted CMDs counter
        _cb.telem[17:18] = list(_cb.hw_status_bits.to_bytes(1, 'big')) #HW Status Bits
        if _cb.rpi_temp_sign:  
            _cb.cpu_temp_msb = 0b00
        else:
            _cb.cpu_temp_msb = 0b01
        rpi_temp_combined_value = (_cb.cpu_temp_msb << 15) | (_cb.rpi_temp & 0x7FFF)
        _cb.telem[18:20] = list(rpi_temp_combined_value.to_bytes(2, 'big'))
        if _cb.temp_sensor_1_sign:  
            _cb.sensor_1_msb = 0b00
        else:
            _cb.sensor_1_msb = 0b01
        temp_sensor_1_value = (_cb.sensor1_temp & 0x7FFF)
        sensor_1_combined_value = ((_cb.sensor_1_msb) << 15) | temp_sensor_1_value  # Sign as MSB
        _cb.telem[20:22] = list(sensor_1_combined_value.to_bytes(2, 'big'))
        _cb.telem[22:24] = list(_cb.cmd_counter_bytes)   #LUCM Cmd counter
        _cb.telem[24:26] = list(_cb.exc_pld_cmd_counter_bytes)   #LUCM Excuted CMDs counter
        _cb.telem[26:27] = list(_cb.last_excuted_cmd.to_bytes(1, 'big'))   #LUCM Cmd counter
        _cb.telem[27:29] = list(_cb.hs_snap_counter.to_bytes(2, 'big'))   #Number of HS snapshots
        _cb.telem[29:31] = list(_cb.vf_snap_counter.to_bytes(2, 'big'))   #Number of VF snapshots
        _cb.telem[31:33] = list(_cb.hs_snap_size.to_bytes(2, 'big'))   #Last Recorded HS Image Size
        _cb.telem[33:35] = list(_cb.compressed_hs_snap_size.to_bytes(2, 'big'))   #Compressed HS Image Size
        # _cb.telem[35:36] = list(_cb.img_compression_type.to_bytes(1, 'big'))   #Compression type
        _cb.telem[36:38] = list(_cb.vf_snap_size.to_bytes(2, 'big'))   #Last Recorded VF Image Size
        # _cb.telem[38:40] = list(_cb.ai_cmd_counter.to_bytes(2, 'big'))   #AI code CMD counter
        _cb.telem[40:42] = list(_cb.boot_cycle_count.to_bytes(2, 'big'))   #LUCM processor boot cycle count
        parity_error = _cb.parity_error
        _cb.telem[42:43] = list(parity_error.to_bytes(1, 'big'))
        _cb.parity_error = 0
        _cb.telem[43:44] = list(_cb.parity_error_counter.to_bytes(1, 'big'))
        stopbit_error = _cb.stopbit_error
        _cb.telem[44:45] = list(stopbit_error.to_bytes(1, 'big'))
        _cb.stopbit_error =0
        _cb.telem[45:46] = list(_cb.stopbit_error_counter.to_bytes(1, 'big'))
        char_spacing_error = _cb.char_spacing_error
        _cb.telem[46:47] = list(char_spacing_error.to_bytes(1, 'big'))
        _cb.char_spacing_error = 0
        _cb.telem[47:48] = list(_cb.char_spacing_error_counter.to_bytes(1, 'big'))
        payload_id_error = _cb.payload_id_error
        _cb.telem[48:49] = list(payload_id_error.to_bytes(1, 'big'))
        _cb.payload_id_error = 0 
        _cb.telem[49:50] = list(_cb.payload_id_error_counter.to_bytes(1, 'big'))
        instruction_type_error = _cb.instruction_type_error
        _cb.telem[50:51] = list(instruction_type_error.to_bytes(1, 'big'))
        _cb.instruction_type_error = 0
        _cb.telem[51:52] = list(_cb.instruction_type_error_counter.to_bytes(1, 'big'))
        instruction_length_error = _cb.instruction_length_error
        _cb.telem[52:53] = list(instruction_length_error.to_bytes(1, 'big'))
        _cb.instruction_length_error = 0
        _cb.telem[53:54] = list(_cb.instruction_length_error_counter.to_bytes(1, 'big'))
        checksum_error = _cb.checksum_error
        _cb.telem[54:55] = list(checksum_error.to_bytes(1, 'big'))
        _cb.checksum_error = 0
        _cb.telem[55:56] = list(_cb.checksum_error_counter.to_bytes(1, 'big'))
        _cb.telem[56:57] = list(current_error_dupe.to_bytes(1, 'big'))
        current_error_dupe = _cb.error_summary
        _cb.telem[57:61] = list(_cb.throttle.to_bytes(4,'big')) # Image processing board throttling status
        _cb.telem[61:63] = list(_cb.ximea_error_value.to_bytes(2,'big')) # Hyperspectral image sensor Error return values
        _cb.telem[63:65] = list(_cb.ram_percentage.to_bytes(2,'big')) # RAM usage percentage
        _cb.telem[65:67] = list(_cb.cpu_percentage.to_bytes(2,'big')) # CPU usage percentage
        # _cb.telem[67:69] = list(_cb.storage_health_percentage.to_bytes(2,'big')) # Storage System Health
        if _cb.temp_sensor_1_sign:  
            _cb.telem[69:70] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[69:70] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[70:72] = list(_cb.sensor1_temp.to_bytes(2,'big')) # Temp sensor1 value Byte
        if _cb.temp_sensor_2_sign:  
            _cb.telem[72:73] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[72:73] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[73:75] = list(_cb.sensor2_temp.to_bytes(2,'big')) # Temp sensor2 value Byte
        if _cb.temp_sensor_3_sign:  
           _cb.telem[75:76] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[75:76] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[76:78] = list(_cb.sensor3_temp.to_bytes(2,'big')) # Temp sensor3 value Byte
        
        if _cb.temp_sensor_4_sign:  
           _cb.telem[78:79] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[78:79] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[79:81] = list(_cb.sensor4_temp.to_bytes(2,'big')) # Temp sensor4 value Byte

        if _cb.temp_sensor_5_sign:  
           _cb.telem[81:82] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[81:82] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[82:84] = list(_cb.sensor5_temp.to_bytes(2,'big')) # Temp sensor5 value Byte

        if _cb.temp_sensor_6_sign:  
           _cb.telem[84:85] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[84:85] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[85:87] = list(_cb.sensor6_temp.to_bytes(2,'big')) # Temp sensor6 value Byte

        if _cb.temp_sensor_7_sign:  
           _cb.telem[87:88] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[87:88] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[88:90] = list(_cb.sensor7_temp.to_bytes(2,'big')) # Temp sensor7 value Byte
        
        if _cb.temp_sensor_8_sign:  
           _cb.telem[90:91] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[90:91] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[91:93] = list(_cb.sensor8_temp.to_bytes(2,'big')) # Temp sensor8 value Byte

        if _cb.temp_sensor_9_sign:  
           _cb.telem[93:94] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[93:94] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[94:96] = list(_cb.sensor9_temp.to_bytes(2,'big')) # Temp sensor9 value Byte

        if _cb.temp_sensor_10_sign:  
           _cb.telem[96:97] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[96:97] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[97:99] = list(_cb.sensor10_temp.to_bytes(2,'big')) # Temp sensor10 value Byte
        if _cb.temp_sensor_11_sign:  
           _cb.telem[99:100] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[99:100] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[100:102] = list(_cb.sensor11_temp.to_bytes(2,'big')) # # Temp sensor11 value Byte

        if _cb.ximea_temp_sign_updated:  
           _cb.telem[102:103] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[102:103] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[103:105] = list(_cb.ximea_temp_value.to_bytes(2,'big')) # Hyperspectral imaging sensor temp
        if _cb.rpi_temp_sign:  
           _cb.telem[105:106] = list(_cb.positive_sign.to_bytes(1,'big'))
        else:
            _cb.telem[105:106] = list(_cb.negative_sign.to_bytes(1,'big'))
        _cb.telem[106:108] = list(_cb.rpi_temp.to_bytes(2,'big')) # Image processing board CPU Temp
        _cb.telem[108:109] = list(_cb.hw_status_bits.to_bytes(1, 'big')) #HW Status Bits
        res.data = _cb.telem
        _cb.telemetry_sequence_count = (_cb.telemetry_sequence_count + 1) % (1 << 14)
        return res

    def data_injection_callback(_cb, req, res):
        _cb.payloadCMD.data = req.data
        _cb.payload_command_publisher.publish(_cb.payloadCMD)
        return res

    def pre_shutdown_callback(_cb, req, res):
        _cb.pre_shutdown = True
        return res

    def default_callback(_cb, req, res):
        return res

    def update_hw_status_for_heaters(_utd, heater1, heater2):
        # Clear bits 2 and 3 (mask 0x0C = 0000 1100)
        _utd.hw_status_bits &= ~0x0C  
        # Set bit 2 if heater1 is active; otherwise it remains 0.
        if heater1:
            _utd.hw_status_bits |= 0x04  # 0x04 = 0000 0100 (sets bit 2)
        # Set bit 3 if heater2 is active; otherwise it remains 0.
        if heater2:
            _utd.hw_status_bits |= 0x08  # 0x08 = 0000 1000 (sets bit 3)

    def update_temperature_data(_utd, msg):
        """Callback to handle received temperature data from /temp topic."""
        # _utd.rpi_temp = msg.rpi_cpu
        _utd.sensor1_temp = msg.temp1
        _utd.sensor2_temp = msg.temp2
        _utd.sensor3_temp = msg.temp3
        _utd.sensor4_temp = msg.temp4
        _utd.sensor5_temp = msg.temp5
        _utd.sensor6_temp = msg.temp6
        _utd.sensor7_temp = msg.temp7
        _utd.sensor8_temp = msg.temp8
        _utd.sensor9_temp = msg.temp9
        _utd.sensor10_temp = msg.temp10
        _utd.sensor11_temp = msg.temp11

        _utd.temp_sensor_1_sign = msg.temp_sign1
        _utd.temp_sensor_2_sign = msg.temp_sign2
        _utd.temp_sensor_3_sign = msg.temp_sign3
        _utd.temp_sensor_4_sign = msg.temp_sign4
        _utd.temp_sensor_5_sign = msg.temp_sign5
        _utd.temp_sensor_6_sign = msg.temp_sign6
        _utd.temp_sensor_7_sign = msg.temp_sign7
        _utd.temp_sensor_8_sign = msg.temp_sign8
        _utd.temp_sensor_9_sign = msg.temp_sign9
        _utd.temp_sensor_10_sign = msg.temp_sign10
        _utd.temp_sensor_11_sign = msg.temp_sign11
        _utd.heater_1_status = msg.heater_1_status
        _utd.heater_2_status = msg.heater_1_status
        _utd.update_hw_status_for_heaters(_utd.heater_1_status, _utd.heater_2_status)

    def update_throttled_status(_uts, msg):
        """Callback to handle received throttled data."""
        _uts.throttle = msg.throttled


    def update_hw_status_for_ram(_utd, ram_percentage):
        # Clear bits 4 and 5 (mask: 0x30 = 0011 0000)
        _utd.hw_status_bits &= ~0x30
        # Set bits 4-5 according to the ram percentage.
        # (0<<4)=0x00, (1<<4)=0x10, (2<<4)=0x20, (3<<4)=0x30
        if ram_percentage <= 20:
            _utd.hw_status_bits |= (0 << 4)
        elif ram_percentage <= 40:
            _utd.hw_status_bits |= (1 << 4)
        elif ram_percentage <= 60:
            _utd.hw_status_bits |= (2 << 4)
        elif ram_percentage <= 80:
            _utd.hw_status_bits |= (3 << 4)

    def update_telem_data(_utd, msg):
        """Callback to handle received telemetry data and update status bits."""
        _utd.rpi_temp_sign = msg.rpi_cpu_sign
        _utd.rpi_temp = msg.rpi_cpu
        _utd.ram_percentage = msg.ram_usage
        _utd.cpu_percentage = msg.cpu_usage
        _utd.storage_health_percentage = int(100-(msg.storage_health))
        _utd.update_hw_status_for_ram(_utd.ram_percentage)

    def update_hw_status_for_cameras(_upt, vf_status, ximea_status):
        # Clear bits 0 and 1 (mask: 0x03 = 0000 0011)
        _upt.hw_status_bits &= ~0x03
        if vf_status:
            _upt.hw_status_bits |= 0x02  # set bit 1
        if ximea_status:
            _upt.hw_status_bits |= 0x01  # set bit 0
    def update_payload_telem_data(_upt,msg):
        _upt.lucm_mode = msg.mode
        _upt.pld_cmd_count =msg.pld_cmd_counter
        _upt.exc_pld_cmd_count = msg.pld_excuted_cmd_counter
        _upt.vf_snap_counter = msg.vf_img_index
        _upt.vf_snap_size = msg.vf_img_size
        _upt.hs_snap_size = msg.hs_img_size
        _upt.compressed_hs_snap_size = msg.compressed_hs_img_size
        _upt.hs_snap_counter = msg.hs_img_index
        _upt.ximea_temp_value = msg.ximea_temp_value
        _upt.ximea_temp_sign_updated = msg.ximea_temp_sign
        _upt.ximea_status = msg.hs_cam_status
        _upt.last_excuted_cmd = msg.last_excuted_command
        _upt.payload_id_error = msg.payload_id_err
        _upt.update_error_instructions(_upt.payload_id_error)
        _upt.update_hw_status_for_cameras(_upt.vf_status, _upt.ximea_status)


    def update_error_data(_upt,msg):
        _upt.update_error_instructions(msg.err)
        pass

    def update_custom_time(_uct):
        if _uct.pre_shutdown:
            _uct.pre_shutdown_msg.data = 's'
            _uct.pre_shutdown_publisher.publish(_uct.pre_shutdown_msg)
            raise SystemExit
        elapsed_real_time = time.time() - _uct.reference_real_time
        _uct.reference_real_time = time.time()
        new_milliseconds = (_uct.custom_time['milliseconds'] + (elapsed_real_time % 1) * 1000)
        extra_seconds = int(new_milliseconds // 1000)
        new_seconds = _uct.custom_time['seconds'] + int(elapsed_real_time) + extra_seconds    
        _uct.custom_time['seconds'] = new_seconds
        _uct.custom_time['milliseconds'] = int(new_milliseconds % 1000)
        _uct.custom_timeMSG.seconds = _uct.custom_time['seconds']
        _uct.custom_timeMSG.milliseconds = _uct.custom_time['milliseconds']
        _uct.custom_time_publisher.publish(_uct.custom_timeMSG)

def main(args=None):
    rclpy.init(args=args)
    try:
        cmd_handler = CmdHandler()
        timer1 = cmd_handler.create_timer(0.1, cmd_handler.update_custom_time)
        # Create a SingleThreadedExecutor and add your node
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(cmd_handler)
        # Spin the executor (this will process callbacks one at a time)
        executor.spin()
    except ValueError as e:
        pass
    finally:
        # Clean up
        executor.shutdown()
        cmd_handler.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
