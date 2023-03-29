from sys import platform

if platform != "win32":
    # Load libprovizio_dds_types.so in the same folder, as Python itself won't look in it
    import ctypes
    import os
    ctypes.cdll.LoadLibrary(os.path.dirname(__file__) + "/libprovizio_dds_types.so")

from provizio_dds_python_types.provizio_dds_python_types import *
