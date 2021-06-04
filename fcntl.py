#!/usr/bin/env micropython
#
# file control and I/O control on file descriptors for micropython
#
# (C) 2021 Peter Züger <zueger.peter@icloud.com>
#
# SPDX-License-Identifier:    MIT
import _libc


# fmt: off
# copied from asm-generic/fcntl.h

FASYNC          = 0o00020000 # fcntl, for BSD compatibility

F_DUPFD         = 0  # dup
F_GETFD         = 1  # get close_on_exec
F_SETFD         = 2  # set/clear close_on_exec
F_GETFL         = 3  # get file->f_flags
F_SETFL         = 4  # set file->f_flags
F_GETLK         = 5
F_SETLK         = 6
F_SETLKW        = 7
F_SETOWN        = 8  # for sockets.
F_GETOWN        = 9  # for sockets.
F_SETSIG        = 10 # for sockets.
F_GETSIG        = 11 # for sockets.

F_GETLK64       = 12 #  using 'struct flock64'
F_SETLK64       = 13
F_SETLKW64      = 14

F_SETOWN_EX     = 15
F_GETOWN_EX     = 16

F_GETOWNER_UIDS = 17

# Open File Description Locks
#
# Usually record locks held by a process are released on *any* close and are
# not inherited across a fork().
#
# These cmd values will set locks that conflict with process-associated
# record  locks, but are "owned" by the open file description, not the
# process. This means that they are inherited across fork() like BSD (flock)
# locks, and they are only released automatically when the last reference to
# the the open file against which they were acquired is put.
F_OFD_GETLK     = 36
F_OFD_SETLK     = 37
F_OFD_SETLKW    = 38

# for F_[GET|SET]FL
FD_CLOEXEC      = 1   # actually anything with low bit set goes

# for posix fcntl() and lockf()
F_RDLCK         = 0
F_WRLCK         = 1
F_UNLCK         = 2

# for old implementation of bsd flock
F_EXLCK         = 4   # or 3
F_SHLCK         = 8   # or 4

# operations for bsd flock(), also used by the kernel implementation
LOCK_SH         = 1   # shared lock
LOCK_EX         = 2   # exclusive lock
LOCK_NB         = 4   # or'd with one of the above to prevent blocking
LOCK_UN         = 8   # remove lock

LOCK_MAND       = 32  # This is a mandatory flock ...
LOCK_READ       = 64  # which allows concurrent read operations
LOCK_WRITE      = 128 # which allows concurrent write operations
LOCK_RW         = 192 # which allows concurrent read & write ops

F_LINUX_SPECIFIC_BASE = 1024


# copied from linux/fcntl.h

F_SETLEASE          = (F_LINUX_SPECIFIC_BASE + 0)
F_GETLEASE          = (F_LINUX_SPECIFIC_BASE + 1)

# Create a file descriptor with FD_CLOEXEC set.
F_DUPFD_CLOEXEC     = (F_LINUX_SPECIFIC_BASE + 6)

# Request nofications on a directory.
# See below for events that may be notified.
F_NOTIFY            = (F_LINUX_SPECIFIC_BASE+2)

# Set and get of pipe page size array
F_SETPIPE_SZ        = (F_LINUX_SPECIFIC_BASE + 7)
F_GETPIPE_SZ        = (F_LINUX_SPECIFIC_BASE + 8)

# Set/Get seals
F_ADD_SEALS         = (F_LINUX_SPECIFIC_BASE + 9)
F_GET_SEALS         = (F_LINUX_SPECIFIC_BASE + 10)

# Types of seals
F_SEAL_SEAL         = 0x0001 # prevent further seals from being set
F_SEAL_SHRINK       = 0x0002 # prevent file from shrinking
F_SEAL_GROW         = 0x0004 # prevent file from growing
F_SEAL_WRITE        = 0x0008 # prevent writes
F_SEAL_FUTURE_WRITE = 0x0010 # prevent future writes while mapped
# (1U << 31) is reserved for signed error codes

# Types of directory notifications that may be requested.
DN_ACCESS           = 0x00000001 # File accessed
DN_MODIFY           = 0x00000002 # File modified
DN_CREATE           = 0x00000004 # File created
DN_DELETE           = 0x00000008 # File removed
DN_RENAME           = 0x00000010 # File renamed
DN_ATTRIB           = 0x00000020 # File changed attibutes
DN_MULTISHOT        = 0x80000000 # Don't remove notifier

# fmt: on


def fcntl(fd, op, arg=0):
    if isinstance(arg, int):
        return _libc.checked_i_fcntl_iil(fd, op, arg)
    return _libc.checked_i_fcntl_iip(fd, op, arg)


def ioctl(fd, op, arg=0, mutate_flag=True):
    if isinstance(arg, int):
        return _libc.checked_i_ioctl_iil(fd, op, arg)
    assert mutate_flag
    return _libc.checked_i_ioctl_iip(fd, op, arg)


def flock(fd, operation):
    return _libc.checked_i_flock_ii(fd, operation)
