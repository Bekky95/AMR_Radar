import sys
import glob
import serial
import re

import logging


def autodetect(ports: list):
    """
    TODO: Change pattern, if your Computer uses different portnumbers than COM7/8 || ACM0/1

    uses port-syntax according to correct os
    COMn --> Win
    ACMn --> Linux

    :param ports: List of available ports
    :return: correct Data/Config Port
    """
    dataPattern = r"(COM7|ACM1)"
    configPattern = r"(COM8|ACM0)"
    result = {}
    for port in ports:
        if re.search(configPattern, port):
            result["config"] = port
            logging.info(f"config {port}")
        elif re.search(dataPattern, port):
            result["data"] = port
            logging.info(f"data {port}")

    return result


def serial_ports():
    """Lists serial port names

    Source - https://stackoverflow.com/a/14224477
    Posted by tfeldmann, modified by community. See post 'Timeline' for change history
    Retrieved 2026-06-03, License - CC BY-SA 3.0

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return autodetect(result)
