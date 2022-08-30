# Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

import pytest
import serial as pyserial
import copy
import serial.tools.list_ports
import time
from typing import Union

DEFAULT_BAUDRATE = 250000

DEFAULT_PORT_CONFIG = {
    'baudrate': DEFAULT_BAUDRATE,
    'bytesize': pyserial.EIGHTBITS,
    'parity': pyserial.PARITY_NONE,
    'stopbits': pyserial.STOPBITS_ONE,
    'timeout': 0.05,  # read timeout
    'xonxoff': False,
    'rtscts': False,
}


def get_serial(port=None, baud=DEFAULT_BAUDRATE, **kwargs) -> pyserial.Serial:
    if port is None:
        ports = serial.tools.list_ports.comports()
        # https://stackoverflow.com/a/52809180
        s = ['\t']
        for port, desc, hwid in sorted(ports):
            s.append("{}: {} [{}]".format(port, desc, hwid))
        s_all = "\n\t".join(s)
        raise ValueError('Please specify port. Available ports:\n' + s_all)

    if isinstance(port, str):
        port_config = copy.deepcopy(DEFAULT_PORT_CONFIG)
        port_config['baudrate'] = baud
        port_config.update(**kwargs)
        proc = pyserial.serial_for_url(port, **port_config)
    else:  # pyserial instance
        proc = port
        proc.timeout = DEFAULT_PORT_CONFIG['timeout']  # set read timeout

    time.sleep(2)  # wait for arduino to start up
    return proc


ardu_ser = None


@pytest.fixture
def ardu():
    global ardu_ser
    if ardu_ser is None:
        ardu_ser = get_serial('COM6')
    return ardu_ser