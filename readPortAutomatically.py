import sys
import glob
import serial
import re


def autodetect(ports: list):
    dataPattern = r"(COM8|ACM1)"
    configPattern = r"(COM7|ACM0)"
    result = {}
    for port in ports:
        if re.search(configPattern, port):
            result["config"] = port
            print("config", port)
        elif re.search(dataPattern, port):
            result["data"] = port
            print("data", port)

    return result


def serial_ports():
    """Lists serial port names

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


if __name__ == "__main__":
    print(serial_ports())
