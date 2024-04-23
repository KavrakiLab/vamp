import ctypes
import os
import sys


class RedirectStream:

    @staticmethod
    def _flush_c_stream(stream):
        try:
            streamname = stream.name[1:-1]
            libc = ctypes.CDLL(None)
            libc.fflush(ctypes.c_void_p.in_dll(libc, streamname))
        except:
            ...

    def __init__(self, stream = sys.stdout, file = os.devnull):
        self.stream = stream
        self.file = file

    def __enter__(self):
        self.stream.flush()                             # ensures python stream unaffected
        self.fd = open(self.file, "w+")
        self.dup_stream = os.dup(self.stream.fileno())
        os.dup2(self.fd.fileno(), self.stream.fileno()) # replaces stream

    def __exit__(self, *_):
        RedirectStream._flush_c_stream(self.stream)    # ensures C stream buffer empty
        os.dup2(self.dup_stream, self.stream.fileno()) # restores stream
        os.close(self.dup_stream)
        self.fd.close()
