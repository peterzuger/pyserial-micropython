#!/usr/bin/env micropython
#
# pySerial adapted for micropython
#
# (C) 2021 Peter Züger <zueger.peter@icloud.com>
#
# SPDX-License-Identifier:    MIT
import uselect as select
import ustruct as struct
import termios

import _libc
import fcntl
import os


if hasattr(termios, "TIOCINQ"):
    TIOCINQ = termios.TIOCINQ
else:
    TIOCINQ = getattr(termios, "FIONREAD", 0x541B)

TIOCM_zero_str = struct.pack("I", 0)

# copied from asm-generic/termbits.h
TCSANOW = getattr(termios, "TCSANOW", 0)
TCSADRAIN = getattr(termios, "TCSADRAIN", 1)
TCSAFLUSH = getattr(termios, "TCSAFLUSH", 2)


class SerialException(OSError):
    """Base class for serial port related exceptions."""


class SerialTimeoutException(SerialException):
    """Write timeouts give an exception"""


class PortNotOpenError(SerialException):
    """Port is not open"""

    def __init__(self):
        super().__init__("Attempting to use a port that is not open")


class Serial:
    # fmt: off
    BAUDRATE_CONSTANTS = {
        0:       0o000000,  # hang up
        50:      0o000001,
        75:      0o000002,
        110:     0o000003,
        134:     0o000004,
        150:     0o000005,
        200:     0o000006,
        300:     0o000007,
        600:     0o000010,
        1200:    0o000011,
        1800:    0o000012,
        2400:    0o000013,
        4800:    0o000014,
        9600:    0o000015,
        19200:   0o000016,
        38400:   0o000017,
        57600:   0o010001,
        115200:  0o010002,
        230400:  0o010003,
        460800:  0o010004,
        500000:  0o010005,
        576000:  0o010006,
        921600:  0o010007,
        1000000: 0o010010,
        1152000: 0o010011,
        1500000: 0o010012,
        2000000: 0o010013,
        2500000: 0o010014,
        3000000: 0o010015,
        3500000: 0o010016,
        4000000: 0o010017,
    }
    # fmt: on

    BAUDRATES = list(BAUDRATE_CONSTANTS.keys())

    def __init__(self, port=None, baudrate=9600, timeout=None, *args, **kwargs):
        self._port = port
        self._baudrate = baudrate
        self._timeout = -1 if timeout is None else int(timeout * 1000)
        self._poller = select.poll()

        self.fd = None
        self.is_open = False

        self.open()

    def open(self):
        if self._port is None:
            raise SerialException("Port must be configured before it can be used.")
        if self.is_open:
            raise SerialException("Port is already open.")

        try:
            self.fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as err:
            self.fd = None
            raise SerialException(
                err.errno, "Could not open port %s: %s" % (self._port, err)
            )

        termios.setraw(self.fd)

        self._poller.register(self.fd, select.POLLIN | select.POLLHUP)

        self.is_open = True

        self.baudrate = self._baudrate

    def close(self):
        if self.is_open:
            if self.fd is not None:
                self._poller.unregister(self.fd)
                os.close(self.fd)
                self.fd = None
            self.is_open = False

    def flush(self):
        if not self.is_open:
            raise PortNotOpenError()
        ret = _libc.i_tcdrain_i(self.fd)
        if ret == -1:
            raise OSError(os.errno())

    def write(self, data):
        if not self.is_open:
            raise PortNotOpenError()
        return os.write(self.fd, data)

    def _read(self, size):
        chunk = os.read(self.fd, size)
        if not chunk:
            # If we read 0 bytes, it means that the port is gone
            # (for example, underlying hardware like USB adapter
            # disconnected)
            raise SerialException("device disconnected")
        return chunk

    def read1(self):
        if not self.is_open:
            raise PortNotOpenError()
        if not self._poller.poll(self._timeout):
            return bytes()
        return self._read(1)

    def read(self, size=1):
        if size == 1:
            return self.read1()
        if not self.is_open:
            raise PortNotOpenError()
        buf = bytes()
        while size > 0:
            if not self._poller.poll(self._timeout):
                break
            chunk = self._read(size)
            size -= len(chunk)
            buf += chunk

        return buf

    @property
    def in_waiting(self):
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCINQ, buf, True)
        return struct.unpack("I", buf)[0]

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self, baudrate):
        if self.is_open:
            _baudrate = self.BAUDRATE_CONSTANTS[baudrate]
            iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(self.fd)
            termios.tcsetattr(
                self.fd,
                TCSADRAIN,
                [iflag, oflag, cflag, lflag, _baudrate, _baudrate, cc],
            )
        self._baudrate = baudrate

    def __enter__(self):
        if self._port is not None and not self.is_open:
            self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
