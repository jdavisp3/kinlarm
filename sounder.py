import subprocess

from config import PLAYBACK_COMMAND, SERIAL_PORT

try:
    import serial
except ImportError:
    pass # No PySerial, SerialSounder will fail


class AudioSounder(object):

    def __init__(self):
        self.child = None

    def activate(self):
        if self.child is None:
            self.child = subprocess.Popen(PLAYBACK_COMMAND, shell=True)

    def deactivate(self):
        if self.child is not None:
            self.child.terminate()
            self.child.wait()
            self.child = None


class SerialSounder(object):

    def __init__(self):
        self.serial = None

    def activate(self):
        if self.serial is None:
            # Just opening the serial port will trigger the DTR line
            self.serial = serial.Serial(SERIAL_PORT)

    def deactivate(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None
