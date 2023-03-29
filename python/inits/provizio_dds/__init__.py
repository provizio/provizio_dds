from sys import platform

if platform != "win32":
    # Load all .so in the same folder, as Python itself won't look in it
    import ctypes
    import os
    module_dir = os.path.dirname(__file__)
    if os.path.isfile(module_dir + "/libfastcdr.so"):
        ctypes.cdll.LoadLibrary(module_dir + "/libfastcdr.so")
    if os.path.isfile(module_dir + "/libfastrtps.so"):
        ctypes.cdll.LoadLibrary(module_dir + "/libfastrtps.so")

from provizio_dds.provizio_dds import *
