import rclpy
from rclpy.node import Node
import psutil
import subprocess
from pl_interface.msg import TelemData, Throttled


class TelemetryData(Node):
    def __init__(self):
        super().__init__('telemetry_data')

        # Declare parameters
        self.declare_parameter('topics.telemetry_data', '/telem')
        self.declare_parameter('topics.throttled', '/throttled')

        # Retrieve the parameter values
        self.telem_topic_name = self.get_parameter('topics.telemetry_data').get_parameter_value().string_value
        self.throttled_topic_name = self.get_parameter('topics.throttled').get_parameter_value().string_value

        # Check if topic names are valid
        if not self.telem_topic_name:
            raise ValueError("Telemetry topic name must not be an empty string.")
        if not self.throttled_topic_name:
            raise ValueError("Throttled topic name must not be an empty string.")

        # Set up publishers
        self.telem_publisher = self.create_publisher(TelemData, self.telem_topic_name, 10)
        self.throttled_publisher = self.create_publisher(Throttled, self.throttled_topic_name, 10)

        # Timers for periodic publishing
        self.create_timer(1.0, self.publish_rpi_telem)      # Publish telemetry every 1 sec
        self.create_timer(1.0, self.publish_throttled_status) # Publish throttled status every 1 sec

    def read_system_param(self):
        """Read system parameters."""
        params = {}
        params['boot_time'] = int(psutil.boot_time())
        mem = psutil.virtual_memory()
        params['memory_usage'] = int(mem.percent)
        disk = psutil.disk_io_counters(perdisk=False, nowrap=True)
        params['write_cycles'] = int(disk.write_count)
        params['cpu_usage'] = int(psutil.cpu_percent(interval=1))
        params['memory_health'] = int((params['write_cycles'] / 100000) * 100)
        return params

    def read_rpi_temp(self):
        """Read the Raspberry Pi CPU temperature."""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_str = f.read()
            return int(temp_str) / 1000.0
        except FileNotFoundError:
            return None

    def get_throttled_status(self):
        """Get the throttled status of the Raspberry Pi."""
        try:
            result = subprocess.run(['vcgencmd', 'get_throttled'],
                                      capture_output=True, text=True, check=True)
            throttled_output = result.stdout.strip()
            if "throttled=" in throttled_output:
                return throttled_output.split('=')[1]
            else:
                return None
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

    def publish_rpi_telem(self):
        """Publish telemetry data."""
        params = self.read_system_param()
        cpu_temp = self.read_rpi_temp()
        def process_temp(value):
            if value is not None:
                return int(abs(value) * 100), value >= 0
            else:
                return 0, True 
        if cpu_temp is None:
            return
        telem_msg = TelemData()
        telem_msg.rpi_cpu, telem_msg.rpi_cpu_sign = process_temp(cpu_temp)
        telem_msg.ram_usage = params['memory_usage']
        telem_msg.cpu_usage = params['cpu_usage']
        telem_msg.storage_health = params['memory_health']
        self.telem_publisher.publish(telem_msg)

    def publish_throttled_status(self):
        """Publish the throttled status of the Raspberry Pi."""
        throttled_status = self.get_throttled_status()
        if throttled_status is None:
            return
        try:
            throttled_value = int(throttled_status, 16)
            throttled_msg = Throttled()
            throttled_msg.throttled = throttled_value
            self.throttled_publisher.publish(throttled_msg)
        except ValueError:
            pass


def main(args=None):
    rclpy.init(args=args)
    telem_data = TelemetryData()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(telem_data)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        telem_data.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
