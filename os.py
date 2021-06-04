#!/usr/bin/env micropython
#
# operating system dependent functionality for micropython
#
# (C) 2021 Peter Züger <zueger.peter@icloud.com>
#
# SPDX-License-Identifier:    MIT
from uos import *

import _libc


# fmt: off
# copied from asm-generic/fcntl.h

O_ACCMODE   = 0o00000003
O_RDONLY    = 0o00000000
O_WRONLY    = 0o00000001
O_RDWR      = 0o00000002
O_CREAT     = 0o00000100 # not fcntl
O_EXCL      = 0o00000200 # not fcntl
O_NOCTTY    = 0o00000400 # not fcntl
O_TRUNC     = 0o00001000 # not fcntl
O_APPEND    = 0o00002000
O_NONBLOCK  = 0o00004000
O_DSYNC     = 0o00010000 # used to be O_SYNC, see below
O_ASYNC     = 0o00020000
O_DIRECT    = 0o00040000 # direct disk access hint
O_LARGEFILE = 0o00100000
O_DIRECTORY = 0o00200000 # must be a directory
O_NOFOLLOW  = 0o00400000 # don't follow links
O_NOATIME   = 0o01000000
O_CLOEXEC   = 0o02000000 # set close_on_exec

__O_SYNC    = 0o04000000
O_SYNC      = (__O_SYNC|O_DSYNC)

O_RSYNC     = O_SYNC # Synchronize read operations.

O_PATH      = 0o010000000

__O_TMPFILE = 0o020000000
O_TMPFILE   = (__O_TMPFILE | O_DIRECTORY)

O_NDELAY    = O_NONBLOCK

# fmt: on


def open(path, flags, mode=0o777, *, dir_fd=None):
    if dir_fd is None:
        return _libc.checked_i_open_sii(path, flags, mode)
    return _libc.checked_i_openat_isii(dir_fd, path, flags, mode)


def close(fd):
    return _libc.checked_i_close_i(fd)


def read(fd, n):
    buf = bytearray(n)
    ret = _libc.checked_i_read_ipi(fd, buf, n)
    return bytes(buf[:ret])


def write(fd, buf):
    return _libc.checked_i_write_iPi(fd, buf, len(buf))
