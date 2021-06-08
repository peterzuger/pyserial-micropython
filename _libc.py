#!/usr/bin/env micropython
#
# libc.so function wrapper with ffi for micropython
#
# (C) 2021 Peter Züger <zueger.peter@icloud.com>
#
# SPDX-License-Identifier:    MIT
import ffi
import uos


def _ffi_open(names):
    err = None
    for name in names:
        try:
            return ffi.open(name)
        except OSError as e:
            err = e
    raise err


def _check_wrapper(fun):
    def checked(*args):
        ret = fun(*args)
        if ret == -1:
            raise OSError(uos.errno())
        return ret

    return checked


_libc_names = ("libc.so",) + tuple("libc.so.%s" % i for i in range(6, -1, -1))
_libc = _ffi_open(_libc_names)
_cache = {}


def __getattr__(attr):
    checked = False
    if attr.startswith("checked_"):
        attr = attr[8:]
        checked = True

    try:
        func = _cache[attr]
    except KeyError:
        rt, name, args = attr.split("_")
        func = _cache[attr] = _libc.func(rt, name, args)

    if checked:
        return _check_wrapper(func)
    return func
