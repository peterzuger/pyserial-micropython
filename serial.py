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

# fmt: off

CR = bytes([13])
LF = bytes([10])

TIOCMGET   = getattr(termios, "TIOCMGET", 0x5415)
TIOCMBIS   = getattr(termios, "TIOCMBIS", 0x5416)
TIOCMBIC   = getattr(termios, "TIOCMBIC", 0x5417)
TIOCMSET   = getattr(termios, "TIOCMSET", 0x5418)

# TIOCM_LE = getattr(termios, 'TIOCM_LE', 0x001)
TIOCM_DTR  = getattr(termios, "TIOCM_DTR", 0x002)
TIOCM_RTS  = getattr(termios, "TIOCM_RTS", 0x004)
# TIOCM_ST = getattr(termios, 'TIOCM_ST', 0x008)
# TIOCM_SR = getattr(termios, 'TIOCM_SR', 0x010)

TIOCM_CTS  = getattr(termios, "TIOCM_CTS", 0x020)
TIOCM_CAR  = getattr(termios, "TIOCM_CAR", 0x040)
TIOCM_RNG  = getattr(termios, "TIOCM_RNG", 0x080)
TIOCM_DSR  = getattr(termios, "TIOCM_DSR", 0x100)
TIOCM_CD   = getattr(termios, "TIOCM_CD",  TIOCM_CAR)
TIOCM_RI   = getattr(termios, "TIOCM_RI",  TIOCM_RNG)

if hasattr(termios, "TIOCINQ"):
    TIOCINQ = termios.TIOCINQ
else:
    TIOCINQ = getattr(termios, "FIONREAD", 0x541B)
TIOCOUTQ = getattr(termios, "TIOCOUTQ", 0x5411)

TIOCM_zero_str = struct.pack("I", 0)
TIOCM_RTS_str  = struct.pack("I", TIOCM_RTS)
TIOCM_DTR_str  = struct.pack("I", TIOCM_DTR)

# copied from asm-generic/termbits.h
TCSANOW   = getattr(termios, "TCSANOW",   0)
TCSADRAIN = getattr(termios, "TCSADRAIN", 1)
TCSAFLUSH = getattr(termios, "TCSAFLUSH", 2)

TIOCSBRK  = getattr(termios, "TIOCSBRK", 0x5427)
TIOCCBRK  = getattr(termios, "TIOCCBRK", 0x5428)

IXON      = getattr(termios, "IXON",    0x400)
IXOFF     = getattr(termios, "IXOFF",   0x1000)
CRTSCTS   = getattr(termios, "CRTSCTS", 0x80000000)

TCIFLUSH  = getattr(termios, "TCIFLUSH", 0)
TCOFLUSH  = getattr(termios, "TCOFLUSH", 1)

CMSPAR = 0o10000000000

CSIZE = getattr(termios, "CSIZE", 0x30)
CS8   = getattr(termios, "CS8",   0x30)
CS7   = getattr(termios, "CS7",   0x20)
CS6   = getattr(termios, "CS6",   0x10)
CS5   = getattr(termios, "CS5",   0x00)

INPCK  = getattr(termios, "INPCK",  0x010)
ISTRIP = getattr(termios, "ISTRIP", 0x020)
PARENB = getattr(termios, "PARENB", 0x100)
PARODD = getattr(termios, "PARODD", 0x200)
CSTOPB = getattr(termios, "CSTOPB", 0x040)

CLOCAL = getattr(termios, "CLOCAL", 0x0800)
CREAD  = getattr(termios, "CREAD",  0x0080)
ICANON = getattr(termios, "ICANON", 0x0002)
ECHO   = getattr(termios, "ECHO",   0x0008)
ECHOE  = getattr(termios, "ECHOE",  0x0010)
ECHOK  = getattr(termios, "ECHOK",  0x0020)
ECHONL = getattr(termios, "ECHONL", 0x0040)
ISIG   = getattr(termios, "ISIG",   0x0001)
IEXTEN = getattr(termios, "IEXTEN", 0x8000)

OPOST  = getattr(termios, "OPOST",  0x001)
ONLCR  = getattr(termios, "ONLCR",  0x004)
OCRNL  = getattr(termios, "OCRNL",  0x008)
INLCR  = getattr(termios, "INLCR",  0x040)
IGNCR  = getattr(termios, "IGNCR",  0x080)
ICRNL  = getattr(termios, "ICRNL",  0x100)
IGNBRK = getattr(termios, "IGNBRK", 0x001)


PARITY_NONE, PARITY_EVEN, PARITY_ODD, PARITY_MARK, PARITY_SPACE = "N","E","O","M","S"
STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO = (1, 1.5, 2)
FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS = (5, 6, 7, 8)

PARITY_NAMES = {
    PARITY_NONE: "None",
    PARITY_EVEN: "Even",
    PARITY_ODD: "Odd",
    PARITY_MARK: "Mark",
    PARITY_SPACE: "Space",
}

# fmt: on


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
    BYTESIZES = (FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS)
    PARITIES = (PARITY_NONE, PARITY_EVEN, PARITY_ODD, PARITY_MARK, PARITY_SPACE)
    STOPBITS = (STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO)

    def __init__(
        self,
        port=None,
        baudrate=9600,
        bytesize=EIGHTBITS,
        parity=PARITY_NONE,
        stopbits=STOPBITS_ONE,
        timeout=None,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        exclusive=None,
        *args,
        **kwargs
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout = -1 if timeout is None else int(timeout * 1000)
        self._poller = select.poll()

        self._bytesize = bytesize
        self._parity = parity
        self._stopbits = stopbits
        self._xonxoff = xonxoff
        self._rtscts = rtscts
        self._dsrdtr = dsrdtr
        self._rts_state = True
        self._dtr_state = True
        self._break_state = False
        self._exclusive = exclusive

        self.fd = None
        self.is_open = False

        if port is not None:
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

        self.exclusive = self._exclusive

        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)

        # set up raw mode / no echo / binary
        cflag |= CLOCAL | CREAD
        lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN)  # |ECHOPRT

        oflag &= ~(OPOST | ONLCR | OCRNL)
        iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK)

        termios.tcsetattr(
            self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
        )

        self._poller.register(self.fd, select.POLLIN | select.POLLHUP)

        self.is_open = True

        self.baudrate = self._baudrate
        self.bytesize = self._bytesize
        self.parity = self._parity
        self.stopbits = self._stopbits
        self.xonxoff = self._xonxoff
        self.rtscts = self._rtscts

        if not self._dsrdtr:
            self.dtr = self._dtr_state

        if not self._rtscts:
            self.rts = self._rts_state

    def close(self):
        if self.is_open:
            self.is_open = False
            if self.fd is not None:
                self._poller.unregister(self.fd)
                os.close(self.fd)
                self.fd = None

    def flush(self):
        if not self.is_open:
            raise PortNotOpenError()
        _libc.checked_i_tcdrain_i(self.fd)

    def reset_input_buffer(self):
        if not self.is_open:
            raise PortNotOpenError()
        _libc.checked_i_tcflush_ii(self.fd, TCIFLUSH)

    def reset_output_buffer(self):
        if not self.is_open:
            raise PortNotOpenError()
        _libc.checked_i_tcflush_ii(self.fd, TCOFLUSH)

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

    def read_until(self, expected=LF, size=None):
        lenterm = len(expected)
        line = bytearray()
        while True:
            c = self.read1()
            if c:
                line += c
                if line[-lenterm:] == expected:
                    break
                if size is not None and len(line) >= size:
                    break
            else:
                break
        return bytes(line)

    @property
    def in_waiting(self):
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCINQ, buf, True)
        return struct.unpack("I", buf)[0]

    @property
    def out_waiting(self):
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCOUTQ, buf, True)
        return struct.unpack("I", buf)[0]

    @property
    def port(self):
        return self._port

    @port.setter
    def port(self, value):
        if value is not None and not isinstance(value, str):
            raise ValueError('"port" must be None or a string, not %s' % type(value))
        was_open = self.is_open
        if was_open:
            self.close()
        self._port = value
        self.name = value
        if was_open:
            self.open()

    @property
    def bytesize(self):
        return self._bytesize

    @bytesize.setter
    def bytesize(self, value):
        if value not in self.BYTESIZES:
            raise ValueError("Not a valid byte size: %r" % value)
        self._bytesize = value
        if self.is_open:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)
            cflag &= ~CSIZE
            if self._bytesize == EIGHTBITS:
                cflag |= CS8
            elif self._bytesize == SEVENBITS:
                cflag |= CS7
            elif self._bytesize == SIXBITS:
                cflag |= CS6
            elif self._bytesize == FIVEBITS:
                cflag |= CS5
            else:
                raise ValueError("Invalid char len: %r" % self._bytesize)
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            )

    @property
    def parity(self):
        return self._parity

    @parity.setter
    def parity(self, value):
        if value not in self.PARITIES:
            raise ValueError("Not a valid parity: %r" % value)
        self._parity = value
        if self.is_open:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)

            # disable input parity checking and input character striping
            iflag &= ~(INPCK | ISTRIP)
            if self._parity == PARITY_NONE:
                cflag &= ~(PARENB | PARODD | CMSPAR)
            elif self._parity == PARITY_EVEN:
                cflag &= ~(PARODD | CMSPAR)
                cflag |= PARENB
            elif self._parity == PARITY_ODD:
                cflag &= ~CMSPAR
                cflag |= PARENB | PARODD
            elif self._parity == PARITY_MARK and CMSPAR:
                cflag |= PARENB | CMSPAR | PARODD
            elif self._parity == PARITY_SPACE and CMSPAR:
                cflag |= PARENB | CMSPAR
                cflag &= ~(PARODD)
            else:
                raise ValueError("Invalid parity: %r" % self._parity)
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            )

    @property
    def stopbits(self):
        return self._stopbits

    @stopbits.setter
    def stopbits(self, value):
        if value not in self.STOPBITS:
            raise ValueError("Not a valid stop bit size: %r" % value)
        self._stopbits = value
        if self.is_open:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)
            if self._stopbits == STOPBITS_ONE:
                cflag &= ~CSTOPB
            elif self._stopbits == STOPBITS_ONE_POINT_FIVE:
                cflag |= CSTOPB
                # XXX same as TWO.. there is no POSIX support for 1.5
            elif self._stopbits == STOPBITS_TWO:
                cflag |= CSTOPB
            else:
                raise ValueError("Invalid stop bit specification: %r" % self._stopbits)
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            )

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self, baudrate):
        if self.is_open:
            try:
                _br = self.BAUDRATE_CONSTANTS[baudrate]
            except KeyError:
                raise ValueError("Invalid baud rate: %r" % baudrate)
            iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(self.fd)
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, _br, _br, cc]
            )
        self._baudrate = baudrate

    @property
    def xonxoff(self):
        return self._xonxoff

    @xonxoff.setter
    def xonxoff(self, value):
        self._xonxoff = value
        if self.is_open:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)
            if self._xonxoff:
                iflag |= IXON | IXOFF
            else:
                iflag &= ~(IXON | IXOFF)
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            )

    @property
    def rtscts(self):
        return self._rtscts

    @rtscts.setter
    def rtscts(self, value):
        self._rtscts = value
        if self.is_open:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.fd)
            if self._rtscts:
                cflag |= CRTSCTS
            else:
                cflag &= ~CRTSCTS
            termios.tcsetattr(
                self.fd, TCSADRAIN, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
            )

    @property
    def dsrdtr(self):
        return self._dsrdtr

    @dsrdtr.setter
    def dsrdtr(self, value):
        raise NotImplementedError("DSR/DTR hardware flow control not implemented")

    @property
    def exclusive(self):
        return self._exclusive

    @exclusive.setter
    def exclusive(self, value):
        if self.is_open:
            if value is not None:
                if value:
                    try:
                        fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except OSError as err:
                        raise SerialException(
                            err.errno,
                            "Could not exclusively lock port %s: %s"
                            % (self._port, err),
                        )
                else:
                    fcntl.flock(self.fd, fcntl.LOCK_UN)
        self._exclusive = value

    @property
    def break_condition(self):
        return self._break_state

    @break_condition.setter
    def break_condition(self, value):
        self._break_state = value
        if self.is_open:
            if self._break_state:
                fcntl.ioctl(self.fd, TIOCSBRK)
            else:
                fcntl.ioctl(self.fd, TIOCCBRK)

    def send_break(self, duration=0.25):
        """\
        Send break condition. Timed, returns to idle state after given
        duration.
        """
        if not self.is_open:
            raise PortNotOpenError()
        _libc.checked_i_tcsendbreak_ii(self.fd, int(duration / 0.25))

    @property
    def rts(self):
        return self._rts_state

    @rts.setter
    def rts(self, value):
        """Set terminal status line: Request To Send"""
        self._rts_state = value
        if self.is_open:
            if self._rts_state:
                fcntl.ioctl(self.fd, TIOCMBIS, TIOCM_RTS_str)
            else:
                fcntl.ioctl(self.fd, TIOCMBIC, TIOCM_RTS_str)

    @property
    def dtr(self):
        return self._dtr_state

    @dtr.setter
    def dtr(self, value):
        """Set terminal status line: Data Terminal Ready"""
        self._dtr_state = value
        if self.is_open:
            if self._dtr_state:
                fcntl.ioctl(self.fd, TIOCMBIS, TIOCM_DTR_str)
            else:
                fcntl.ioctl(self.fd, TIOCMBIC, TIOCM_DTR_str)

    @property
    def cts(self):
        """Read terminal status line: Clear To Send"""
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCMGET, buf, True)
        return struct.unpack("I", buf)[0] & TIOCM_CTS != 0

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready"""
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCMGET, buf, True)
        return struct.unpack("I", buf)[0] & TIOCM_DSR != 0

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator"""
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCMGET, buf, True)
        return struct.unpack("I", buf)[0] & TIOCM_RI != 0

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect"""
        if not self.is_open:
            raise PortNotOpenError()
        buf = struct.pack("I", 0)
        fcntl.ioctl(self.fd, TIOCMGET, buf, True)
        return struct.unpack("I", buf)[0] & TIOCM_CD != 0

    # compatibility with io library

    def readable(self):
        return True

    def writable(self):
        return True

    def seekable(self):
        return False

    def readinto(self, b):
        data = self.read(len(b))
        n = len(data)
        try:
            b[:n] = data
        except TypeError as err:
            import uarray as array

            if not isinstance(b, array.array):
                raise err
            b[:n] = array.array("b", data)
        return n

    def readline(self, size=-1):
        if size is None:
            size = -1
        line = bytearray()
        while size < 0 or len(line) < size:
            b = self.read1()
            if not b:
                break
            line += b
            if line.endswith(b"\n"):
                break
        return bytes(line)

    def readlines(self, hint=-1):
        if hint is None or hint <= 0:
            return list(self)
        n = 0
        lines = []
        for line in self:
            lines.append(line)
            n += len(line)
            if n >= hint:
                break
        return lines

    def writelines(self, lines):
        for line in lines:
            self.write(line)

    def __iter__(self):
        if not self.is_open:
            raise PortNotOpenError()
        return self

    def __next__(self):
        line = self.readline()
        if not line:
            raise StopIteration
        return line

    # context manager

    def __enter__(self):
        if self._port is not None and not self.is_open:
            self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # platform specific

    def fileno(self):
        if not self.is_open:
            raise PortNotOpenError()
        return self.fd
