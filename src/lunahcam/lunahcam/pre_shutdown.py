"""
-   Node named "pre_shutdown" used for executing pre-shutdown command procedures
-   inherits from Node & LogLevel Classes
-   runs "sudo shutdown now" bash command to prepare for power cut-off
-   listens on a topic named /shutdown (defined in config.yaml file) with an std.msg type of "String"
-   if msg content is 's' it executes the command
"""

# Necessary Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import subprocess 

# directory mapping for importing local files
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from log_level import LogLevel

class PreShutdown(Node,LogLevel):

    # class constructor
    def __init__(_psh):
        super().__init__('pre_shutdown') # initializing of Node class (one of the parent classes)
        LogLevel.__init__(_psh) # initializing of LogLevel class (one of the parent classes)
        
        # creation of subscriber that listens on /shutdown topic 
        # "pre_shutdown_callback" as the callback function and std.msg String as msg type
        _psh.pre_shutdown_subscriber = _psh.create_subscription(
            String,
            '/shutdown',
            _psh.pre_shutdown_callback,
            10)
        
    # Logging function
    """
    -   A Function to print cutom logging messages for debugging purposes only
    -   Please remove any debugging/logging statements in production
    -   args:
        -   type: (err,warn,nok,ok,info) arg_type: string
        -   log: (msg to be logged) arg_type: string
    """
    # def log(_log,type,log):
    #     match type:
    #         case "err":
    #             _log.get_logger().info(f'{_log.red}{log}{_log.reset}')
    #         case 'warn':
    #             _log.get_logger().info(f'{_log.yellow}{log}{_log.reset}')
    #         case 'nok':
    #             _log.get_logger().info(f'{_log.pink}{log}{_log.reset}')  
    #         case 'ok':
    #             _log.get_logger().info(f'{_log.cyan}{log}{_log.reset}')
    #         case 'info':
    #             _log.get_logger().info(f'{_log.green}{log}{_log.reset}')
    #         case _:
    #             _log.get_logger().info(f'{_log.darkgrey}{log}{_log.reset}')
                
    # pre_shutdown callback function
    """
    -   A callback function to be invoked when a msg is published on /shutdown topic
    -   args:
        -   msg: the content of the recieved message (accessed like this: msg.data) arg_type: string
    """
    def pre_shutdown_callback(_cb, msg):
        try:
            # _cb.log('info',f"{msg.data}")
            if msg.data == 's': # if message content is 's'
                # _cb.log('info',"executing shutdown command...")
                subprocess.run(['sudo','shutdown', '-h', 'now'], check=True) # execute a shutdown bash command in a separate subprocess
                raise SystemExit # raise a SystemExit to shutdown the node
        except Exception as e:
            # _cb.log('err',f'{e}')
            pass


def main(args=None):
    try:
        rclpy.init(args=args)

        pre_shutdown_subscriber = PreShutdown()

        rclpy.spin(pre_shutdown_subscriber)

        pre_shutdown_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        raise SystemExit


if __name__ == '__main__':
    main()