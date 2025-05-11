import rclpy
from rclpy.node import Node
import gpiod
import sys
import os
import time
import glob

#current_dir = os.path.dirname(os.path.abspath(__file__))
current_dir = "/LUNAHCAM_V1/LUNAHCAM/src/lunahcam"
sys.path.append(current_dir)

from pl_interface.msg import Temp, TargetTemp

class ThermalControl(Node):
    def __init__(self):
        super().__init__('thermal_control')

        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')

        # Initialize sensors with specific addresses
        self.addr_T1 = "28-0122931dadf5"  # power board temp
        self.addr_T2 = "28-000010713f79"  # z-
        self.addr_T3 = "28-000010722206"  # y-
        self.addr_T4 = "28-00001072486b"  # Z-
        self.addr_T5 = "28-00001071fdc1"  # ximea
        self.addr_T6 = "28-000010711539"  # Y+
        self.addr_T7 = "28-000010715b65"  # Y-
        self.addr_T8 = "28-000010711efc"  # optical
        self.addr_T9 = "28-00001072360f"  # optical
        self.addr_T10 = "28-000010712e70" # X+ OSR
        self.addr_T11 = "28-00001070e4ab" # X+ MLI

        self.actual_t1 = None
        self.actual_t2 = None
        self.actual_t3 = None
        self.actual_t4 = None
        self.actual_t5 = None
        self.actual_t6 = None
        self.actual_t7 = None
        self.actual_t8 = None
        self.actual_t9 = None
        self.actual_t10 = None
        self.actual_t11 = None

        self.pin_G1 = 23
        self.pin_G2 = 24
        self.target_T = 20.0
        self.target_temp_value = None
        self.target_temp_sign = None

        self.base_dir = '/sys/bus/w1/devices/'
        self.device_folder = glob.glob(self.base_dir + '28*')
        self.heater_1_status = False
        self.heater_2_status = False

        # Configure GPIO for heaters
        self.chip = gpiod.Chip('/dev/gpiochip0')
        self.line1 = self.chip.get_line(self.pin_G1)
        self.line2 = self.chip.get_line(self.pin_G2)
        self.line1.request(consumer='heater_control', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self.line2.request(consumer='heater_control2', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

        # ROS 2 parameters and topics
        self.declare_parameter('topics.temp', '/temp')
        self.temp_topic_name = self.get_parameter('topics.temp').get_parameter_value().string_value
        self.declare_parameter('topics.target_heater_temp', '/target_heater_temp')
        self.target_heater_temp_topic_name = self.get_parameter('topics.target_heater_temp').get_parameter_value().string_value

        self.temps_publisher = self.create_publisher(Temp, self.temp_topic_name, 10)
        self.target_temp_subscriber = self.create_subscription(TargetTemp,
                                                               self.target_heater_temp_topic_name,
                                                               self.get_target_temp,
                                                               10)
        self.create_timer(0.1, self.thermal_control_callback)
        self.create_timer(1.0, self.publish_sensor_temp)
        self.start_time = time.time()

    def get_target_temp(self, msg):
        """
        Callback for receiving target temperature messages.
        """
        self.target_temp_value = float(msg.target_temp_value)
        self.target_temp_sign = msg.target_temp_sign
        if self.target_temp_value is not None and (0.0 < self.target_temp_value <= 45.0) and self.target_temp_sign:
            self.target_T = self.target_temp_value

    def read_temp_files(self, t_addrs):
        lines = []
        try:
            with open(self.base_dir + t_addrs[0] + '/w1_slave', 'r') as t1:
                lines.append(t1.readlines())
            return lines
        except Exception:
            self.line1.set_value(1)
            self.line2.set_value(1)
            self.heater_1_status = False
            self.heater_2_status = False
            return [['ff ff ff ff ff ff ff ff ff : crc=ff NOO\n', 'ff ff ff ff ff ff ff ff ff t=99999\n']]
        except KeyboardInterrupt:
            raise SystemExit

    def read_actual_temp(self, t_addrs):
        try:
            lines = self.read_temp_files(t_addrs)
            crc_check = lines[0][0].strip()[-3:]
            if crc_check == 'YES':
                equal_pos_1 = lines[0][1].find('=')
                actual_t = round(float(lines[0][1][equal_pos_1+1:]) / 1000, 1)
                return actual_t
            else:
                return None
        except Exception:
            self.line1.set_value(1)
            self.line2.set_value(1)
            self.heater_1_status = False
            self.heater_2_status = False
            return None
        except KeyboardInterrupt:
            raise SystemExit

    def set_heater_test(self):
        if self.target_temp_value is not None:
            if 0.0 < self.target_temp_value <= 45.0:
                self.target_T = self.target_temp_value

    def thermal_control_callback(self):
        """
        Timer callback to perform one iteration of the thermal control loop.
        """
        # Read sensor temperatures
        self.actual_t1 = self.read_actual_temp([self.addr_T1])
        self.actual_t2 = self.read_actual_temp([self.addr_T2])
        self.actual_t3 = self.read_actual_temp([self.addr_T3])
        self.actual_t4 = self.read_actual_temp([self.addr_T4])
        self.actual_t5 = self.read_actual_temp([self.addr_T5])
        self.actual_t6 = self.read_actual_temp([self.addr_T6])
        self.actual_t7 = self.read_actual_temp([self.addr_T7])
        self.actual_t8 = self.read_actual_temp([self.addr_T8])
        self.actual_t9 = self.read_actual_temp([self.addr_T9])
        self.actual_t10 = self.read_actual_temp([self.addr_T10])
        self.actual_t11 = self.read_actual_temp([self.addr_T11])

        self.set_heater_test()
        if self.actual_t8 is not None and self.actual_t9 is not None:
            if self.actual_t8 > self.target_T or self.actual_t9 > self.target_T:
                self.line1.set_value(1)
                self.line2.set_value(1)
                self.heater_1_status = False
                self.heater_2_status = False
            else:
                self.line1.set_value(0)
                self.line2.set_value(0)
                self.heater_1_status = True
                self.heater_2_status = True

    def publish_sensor_temp(self):
        msg = Temp()
        # Define a helper function to process temperatures
        def process_temp(value):
            if value is not None:
                return int(abs(value) * 100), value >= 0
            else:
                return 0, True
        msg.temp1, msg.temp_sign1 = process_temp(self.actual_t1)
        msg.temp2, msg.temp_sign2 = process_temp(self.actual_t2)
        msg.temp3, msg.temp_sign3 = process_temp(self.actual_t3)
        msg.temp4, msg.temp_sign4 = process_temp(self.actual_t4)
        msg.temp5, msg.temp_sign5 = process_temp(self.actual_t5)
        msg.temp6, msg.temp_sign6 = process_temp(self.actual_t6)
        msg.temp7, msg.temp_sign7 = process_temp(self.actual_t7)
        msg.temp8, msg.temp_sign8 = process_temp(self.actual_t8)
        msg.temp9, msg.temp_sign9 = process_temp(self.actual_t9)
        msg.temp10, msg.temp_sign10 = process_temp(self.actual_t10)
        msg.temp11, msg.temp_sign11 = process_temp(self.actual_t11)
        msg.heater_1_status = self.heater_1_status
        msg.heater_2_status = self.heater_2_status
        self.temps_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    thermal_control = ThermalControl()

    # Run the node using a SingleThreadedExecutor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(thermal_control)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        thermal_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
