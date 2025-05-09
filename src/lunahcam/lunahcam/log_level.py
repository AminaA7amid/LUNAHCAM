""" 
A Custom class for printing/logging cutom logging messages 
with different logging levels.
"""
class LogLevel:
    def __init__(self):

        self.reset = '\033[0m'         # Reset color and formatting
        self.bold = '\033[01m'         # Bold
        self.disable = '\033[02m'      # Disable
        self.underline = '\033[04m'    # Underline
        self.reverse = '\033[07m'      # Reverse
        self.strikethrough = '\033[09m'# Strikethrough
        self.invisible = '\033[08m'    # Invisible
        
        self.black = '\033[30m'        # Black
        self.red = '\033[31m'          # Red
        self.green = '\033[32m'        # Green
        self.orange = '\033[33m'       # Orange
        self.blue = '\033[34m'         # Blue
        self.purple = '\033[35m'       # Purple
        self.cyan = '\033[36m'         # Cyan
        self.lightgrey = '\033[37m'    # Light Grey
        self.darkgrey = '\033[90m'     # Dark Grey
        self.lightred = '\033[91m'     # Light Red
        self.lightgreen = '\033[92m'   # Light Green
        self.yellow = '\033[93m'       # Yellow
        self.lightblue = '\033[94m'    # Light Blue
        self.pink = '\033[95m'         # Pink
        self.lightcyan = '\033[96m'    # Light Cyan
