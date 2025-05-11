import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import serial
from serial import SerialException
import sys
import os
import time
import threading
import gpiod
import subprocess

from std_msgs.msg import String
from pl_interface.msg import Error
from pl_interface.srv import BusReply
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


#current_dir = os.path.dirname(os.path.abspath(__file__))
current_dir = "/LUNAHCAM_V1/LUNAHCAM/src/lunahcam"

sys.path.append(current_dir)

from protocol import Protocol
from log_level import LogLevel

class BusInterface(Node, Protocol, LogLevel):
    def __init__(self, addr):
        super().__init__('bus_interface')
        Protocol.__init__(self)
        LogLevel.__init__(self)
        self._allow_undeclared_parameters = True
        self.namespace = self.get_namespace()
        self.ns = self.namespace[1:]

        self.declare_parameters(
            namespace=self.namespace,
            parameters=[
                (f'uart_config.port', ""),
                ('uart_config.baud_rate', 0),
                ('uart_config.parity', ""),
                ('uart_config.stop', 0),
                ('uart_config.data', 0),
                ('uart_config.timeout', 0.0),
                ('uart_config.control_pin', 0),
                ('services.cmd_srv', ""),
                ('shutdown.shutdown', ""),
                ('topics.error', "/error")
            ]
        )
        
        self.portUART = self.get_parameter(f'{self.namespace}.uart_config.port').value
        self.baudRATE = self.get_parameter(f'{self.namespace}.uart_config.baud_rate').value
        self.PARITY = self.get_parameter(f'{self.namespace}.uart_config.parity').value 
        self.stopBITS = self.get_parameter(f'{self.namespace}.uart_config.stop').value 
        self.byteSIZE = self.get_parameter(f'{self.namespace}.uart_config.data').value 
        self.TIMEOUT = self.get_parameter(f'{self.namespace}.uart_config.timeout').value
        self.controlPIN = self.get_parameter(f'{self.namespace}.uart_config.control_pin').value
        
        self.cmd_service_name = self.get_parameter(f'{self.namespace}.services.cmd_srv').value
        self.shutdown_topic_name = self.get_parameter(f'{self.namespace}.topics.shutdown').value
        self.error_topic_name = self.get_parameter(f'{self.namespace}.topics.error').value
        self.error_command_publisher = self.create_publisher(Error, f'{self.error_topic_name}', 10)
        self.addr = addr
        self.transmit_frame_duration = 0
        self.recieve_frame_duration = 0
        
        self.SER = None
        # Create a lock for shared resources (e.g. self.SER)
        self.lock = threading.Lock()

        # Release any existing line references
        if hasattr(self, 'line'):
            try:
                self.line.release()
            except Exception as e:
                self.line = None
        
        if hasattr(self, 'chip'):
            try:
                self.chip.close()
            except Exception as e:
                self.chip = None
        
        self.chip = gpiod.Chip('gpiochip0')
        self.line = self.chip.get_line(self.controlPIN)
        self.line.request(consumer='bus_interface', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bus_client = self.create_client(BusReply, self.cmd_service_name, qos_profile=qos_profile)
        self.req = BusReply.Request()
        
        self.shutdown_publisher = self.create_publisher(String, f'/{self.shutdown_topic_name}', 10)
        # Optionally, a flag to signal shutdown in the standby loop:
        self.shutdown_flag = threading.Event()


    def publish_error_codes(self, err):
        error_message = Error()
        error_message.err = err
        self.error_command_publisher.publish(error_message)

    def send_request(self, cmd, data, response_callback):
        """
        Prepare and send a service request. Instead of busy waiting,
        we register a callback to process the service response.
        """
        if hasattr(self, 'errCODE') and self.errCODE:
            self.req.cmd = cmd
            self.req.err = self.errCODE
            self.publish_error_codes(self.req.err)
            self.req.data_len = 0
            self.req.data = []
        else:
            self.req.cmd = cmd
            self.req.data_len = len(data) - 1
            self.req.data = data
            self.req.err = 0

        future = self.bus_client.call_async(self.req)
        future.add_done_callback(response_callback)
        return future


    def open_serial_port(self):
        try:
            with self.lock:
                if self.SER is not None and self.SER.is_open:
                    self.SER.close()
                self.SER = serial.Serial(
                    port=self.portUART,
                    baudrate=self.baudRATE,
                    parity=self.PARITY,
                    stopbits=self.stopBITS,
                    bytesize=self.byteSIZE,
                    timeout=self.TIMEOUT
                )
        except SerialException as e:
            self.get_logger().error(f"Serial exception: {e}")
        except KeyboardInterrupt:
            raise SystemExit


    def transmit_frame(self, frame):
        start_time = time.perf_counter()
        self.line.set_value(1)
        try:
            with self.lock:
                # Wait until pending output is transmitted
                # while self.SER.out_waiting:
                #     time.sleep(0.001)
                self.SER.write(bytearray(frame))
            end_time = time.perf_counter()
            self.transmit_frame_duration = (end_time - start_time)*1000
        except SerialException as e:
            self.line.set_value(0)
            if hasattr(self, 'errTX'):
                self.errCODE = self.errTX
        except KeyboardInterrupt:
            self.line.set_value(0)
            raise SystemExit


    def recieve_frame(self):
        """
        Reads a complete frame from the serial port, then depacketizes it.
        If an error is detected, publish the error and return (False, {}).
        Otherwise, return (True, depacketized_frame).
        """
        with self.lock:
            self.SER.flush()
        self.line.set_value(0)
        frame = b""
        try:
            # Wait until data is incoming
            # while not self.SER.in_waiting:
            #     time.sleep(0.001)
            # Read the frame in pieces: first 5 bytes then the rest based on the length
            start_time = time.perf_counter()
            while self.SER.in_waiting:
                temp = self.SER.read(5)
                data_length = temp[-1] + 1
                frame += temp
                temp = self.SER.read(data_length + 2)
                frame += temp

            depacketized_frame = self.depacketize(frame)
            end_time = time.perf_counter()
            self.recieve_frame_duration = (end_time - start_time)

            if ("err_code" in depacketized_frame and
                    depacketized_frame["err_code"] != self.errors["0"]):
                self.errCODE = int(list(self.errors.keys())[
                    list(self.errors.values()).index(depacketized_frame["err_code"])
                ])
                self.publish_error_codes(self.errCODE)
                return False, {}

            return True, depacketized_frame

        except AssertionError as e:
            with self.lock:
                self.SER.reset_input_buffer()
            self.errCODE = e.args[0] if e.args else self.errFRAME
            self.publish_error_codes(self.errCODE)
            self.line.set_value(0)
            return False, {}
        except SerialException as e:
            self.errCODE = self.errRX
            self.publish_error_codes(self.errCODE)
            self.line.set_value(0)
            return False, {}
        except KeyboardInterrupt:
            self.line.set_value(0)
            raise SystemExit
        
    def standby(self):
        """
        Main loop that waits for an incoming frame, sends a service request,
        and then handles the service response via an asynchronous callback.
        """
        while not self.shutdown_flag.is_set():
            try:
                # 1) Wait for and parse an incoming frame
                reception_status, instruction_frame = self.recieve_frame()
                # Record time after successful reception
                start_time = time.perf_counter()
                if not reception_status or instruction_frame.get("res_code", 0) == self.errorINSTRUCTION:
                    error_response = self.packetize(self.errorINSTRUCTION)
                    self.transmit_frame(error_response)
                    continue


                # Use an event to wait for the service response processing if needed.
                response_processed = threading.Event()

                # Define a callback to process the service response
                def response_callback(future, frame=instruction_frame, start=start_time, event=response_processed):
                    try:
                        response = future.result()
                        # Process the response based on the command received:
                        match frame["cmd"]:
                            case self.cmdTIME_SYNC:
                                # Process time synchronization if applicable
                                pass

                            case self.cmdTELEM_POLLING:
                                rply_frame = self.packetize(response.res_code, response.data)
                                self.transmit_frame(rply_frame)
                                end_time = time.perf_counter()
                                latency = (end_time - start)*1000.0
                                # self.get_logger().info(f"TELEM_POLLING Latency: {latency:.6f} ms")
                                # self.time_logger()
                                # self.get_logger().info(
                                #     f"recieve_frame_duration: {self.recieve_frame_duration:.6f} ms")
                                # self.get_logger().info(
                                #     f"transmit_frame_duration: {self.transmit_frame_duration:.6f} ms")
                            case self.cmdPRE_SHUTDOWN:
                                rply_frame = self.packetize(response.res_code)
                                self.transmit_frame(rply_frame)
                                msg = String()
                                msg.data = 's'
                                self.shutdown_publisher.publish(msg)
                                raise SystemExit
                            
                            case self.cmdDATA_INJECTION:
                                rply_frame = self.packetize(response.res_code)
                                self.transmit_frame(rply_frame)
                                end_time = time.perf_counter()
                                latency = (end_time - start)*1000
                                # self.get_logger().info(f"DATA_INJECTION Latency: {latency:.6f} ms")
                                # self.time_logger()
                                # self.get_logger().info(
                                #     f"recieve_frame_duration: {self.recieve_frame_duration:.6f} ms")
                                # self.get_logger().info(
                                #     f"transmit_frame_duration: {self.transmit_frame_duration:.6f} ms")
                            case _:
                                end_time = time.perf_counter()
                                latency = (end_time - start)*1000
                                # self.get_logger().info(f"Unknown CMD Latency: {latency:.6f} ms")
                                # self.time_logger()
                                # self.get_logger().info(
                                #     f"recieve_frame_duration: {self.recieve_frame_duration:.6f} ms")
                                # self.get_logger().info(
                                #     f"transmit_frame_duration: {self.transmit_frame_duration:.6f} ms")
                    except Exception as e:
                        error_response = self.packetize(self.errorINSTRUCTION)
                        self.transmit_frame(error_response)
                    finally:
                        event.set()  # signal that response processing is done

                # 2) Send the service request and register the callback
                self.send_request(instruction_frame["cmd"], instruction_frame["data"], response_callback)

                # Wait until the callback signals that processing is complete
                response_processed.wait()

            except Exception as e:
                pass
                # self.get_logger().error(f"Error in standby loop: {e}")
                # Optionally, include a short delay to prevent spinning on error
                # time.sleep(0.01)


def main(args=None):
    try:
        rclpy.init(args=args)
        bus_interface = BusInterface(0x06)

        # Start the ROS executor in its own thread.
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bus_interface)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Open serial port before starting the standby loop.
        bus_interface.open_serial_port()

        # Run the blocking standby loop in its own thread.
        standby_thread = threading.Thread(target=bus_interface.standby, daemon=True)
        standby_thread.start()

        # Optionally, wait for the standby thread to finish if you need a join.
        standby_thread.join()
        executor_thread.join()

    except SystemExit:
        if bus_interface.SER is not None and bus_interface.SER.is_open:
            bus_interface.SER.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
