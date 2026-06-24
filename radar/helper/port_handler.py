import serial


class PortHandler:
    def __init__(self, cur_port: serial.Serial):
        self.current_port = cur_port

    def serial_config(self):
        pass

    def parse_config_file(self):
        pass

    def read_port(self):
        pass


class DataPortHandler(PortHandler):
    def __init__(self):
        self.cur_data_port = serial.Serial()
        super().__init__(self.cur_data_port)
        pass


class CliPortHandler(PortHandler):
    def __init__(self):
        self.cur_cli_port = serial.Serial()
        super().__init__(self.cur_cli_port)
        pass
