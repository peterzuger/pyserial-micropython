#!/usr/bin/env micropython
#
# libc.so function wrapper with ffi for micropython
#
# (C) 2021 Peter Züger <zueger.peter@icloud.com>
#
# SPDX-License-Identifier:    MIT
import ffi


def _ffi_open(names):
    err = None
    for name in names:
        try:
            return ffi.open(name)
        except OSError as e:
            err = e
    raise err


_libc_names = ("libc.so",) + tuple("libc.so.%s" % i for i in range(6, -1, -1))
_libc = _ffi_open(_libc_names)
_cache = dict()


def __getattr__(attr):
    try:
        func = _cache[attr]
    except KeyError:
        rt, name, args = attr.split("_")
        func = _cache[attr] = _libc.func(rt, name, args)
    return func
