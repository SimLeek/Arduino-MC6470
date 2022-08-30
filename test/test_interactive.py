# Copyright (C) 2022 - Simleek <simulatorleek@gmail.com> - MIT License

from ardu_pytest_fixture import ardu
import time


def test_serial_works(ardu):
    ardu.write(b'B')
    a = ardu.read(3)
    assert a == b'Ack'


def test_mag_loop(ardu, capsys):
    t0 = time.time()
    with capsys.disabled():
        ardu.write(b'a')  # start mag loop
        while time.time() < t0 + 10:
            #ardu.write(b'a')  # start mag loop
            print(ardu.readline())
        ardu.write(b'b')  # stop mag loop
        print(ardu.readlines())  # print remainder until timeout


def test_accel_loop(ardu, capsys):
    t0 = time.time()
    with capsys.disabled():
        ardu.write(b'c')  # start acc loop
        while time.time() < t0 + 10:
            print(ardu.readline())
        ardu.write(b'd')  # stop acc loop
        print(ardu.readlines())  # print remainder until timeout


def test_fusion_loop(ardu, capsys):
    t0 = time.time()
    with capsys.disabled():
        ardu.write(b'a')  # start acc loop
        ardu.write(b'c')  # start acc loop
        ardu.write(b'e')  # start acc loop
        while time.time() < t0 + 10:
            print(ardu.readline())
        ardu.write(b'b')  # stop mag loop
        ardu.write(b'd')  # stop acc loop
        ardu.write(b'f')  # start acc loop
        print(ardu.readlines())  # print remainder until timeout


def test_set_rate(ardu, capsys):
    ardu.write(b's')
    ardu.write(bytes(chr(0b00001010), encoding='ascii'))  # write 256hz
    with capsys.disabled():
        line = ardu.readline()
        print(line)
    ardu.write(b'u')
    with capsys.disabled():
        line = ardu.readline()
        print(line)
        assert line == b"Got acc rate: 10\r\n"


def test_temp_loop(ardu, capsys):
    t0 = time.time()
    with capsys.disabled():
        ardu.write(b't')  # start temp loop
        while time.time() < t0 + 10:
            ardu.write(b't')  # start temp loop
            print(ardu.readline())
        ardu.write(b'b')  # stop mag/temp loop
        print(ardu.readlines())  # print remainder until timeout


def test_acc_set_gain_and_offset(ardu, capsys):
    ardu.write(b'w')
    with capsys.disabled():
        line1 = ardu.readline()
        line2 = ardu.readline()
        line3 = ardu.readline()
        print(line1)
        print(line2)
        print(line3)

        # ardu.write('v')
        # todo: no, get the actual offset/gain and add and subtract one, so we don't screw up the chip
        # ardu.write("".join([chr(i) for i in range(18)]))  # write random gain/offset


def test_mag_set_gain_and_offset(ardu, capsys):
    ardu.write(b'A')
    with capsys.disabled():
        line1 = ardu.readline()
        print(line1)

        # ardu.write('z')
        # todo: no, get the actual offset/gain and add and subtract one, so we don't screw up the chip
        # ardu.write("".join([chr(i) for i in range(18)]))  # write random gain/offset


def test_set_acc_res_range(ardu, capsys):
    ardu.write(b'x')
    ardu.write(bytes(chr(0b00000101), encoding='ascii'))  # set 14 bit res
    ardu.write(bytes(chr(0b00010000), encoding='ascii'))  # set 4g range
    with capsys.disabled():
        line1 = ardu.readline()
        print(line1)
        assert line1 == b"Acc res and range were set successfully.\r\n"

        ardu.write(b'y')

        line1 = ardu.readline()
        print(line1)
        #print(ardu.readlines())
        assert line1 == f"Got acc res and range: {0b00000101 | 0b00010000}\r\n".encode()
