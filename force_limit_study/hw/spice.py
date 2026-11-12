"""Small synchronous ngspice shared-library interface; no external services."""
import ctypes as ct
import os
from pathlib import Path
import numpy as np

class Vector(ct.Structure):
    _fields_ = [('name', ct.c_char_p), ('type', ct.c_int), ('flags', ct.c_short),
                ('real', ct.POINTER(ct.c_double)), ('complex', ct.c_void_p), ('length', ct.c_int)]

class Spice:
    def __init__(self):
        candidates = [os.environ.get('NGSPICE_DLL'),
                      Path(os.environ.get('LOCALAPPDATA', '')) /
                      'Programs/KiCad/10.0/bin/ngspice.dll',
                      Path(os.environ.get('ProgramFiles', '')) /
                      'KiCad/10.0/bin/ngspice.dll']
        errors = []
        for candidate in candidates:
            if not candidate or not Path(candidate).is_file():
                continue
            try:
                self.path = Path(candidate)
                self.directory = os.add_dll_directory(str(self.path.parent))
                self.lib = ct.CDLL(str(self.path))
                break
            except OSError as error:
                errors.append(str(error))
        else:
            raise OSError('Cannot load ngspice: ' + '; '.join(errors))
        self.log = []
        send = ct.CFUNCTYPE(ct.c_int, ct.c_char_p, ct.c_int, ct.c_void_p)
        status = ct.CFUNCTYPE(ct.c_int, ct.c_char_p, ct.c_int, ct.c_void_p)
        leave = ct.CFUNCTYPE(ct.c_int, ct.c_int, ct.c_bool, ct.c_bool, ct.c_int, ct.c_void_p)
        self.callbacks = [send(lambda msg, *_: self.log.append(msg.decode(errors='replace')) or 0),
                          status(lambda *_: 0), leave(lambda *_: 0)]
        self.lib.ngSpice_Init.argtypes = [send,status,leave,ct.c_void_p,ct.c_void_p,ct.c_void_p,ct.c_void_p]
        self.lib.ngSpice_Init(*self.callbacks, None,None,None,None)
        self.lib.ngSpice_Command.argtypes = [ct.c_char_p]
        self.lib.ngSpice_Circ.argtypes = [ct.POINTER(ct.c_char_p)]
        self.lib.ngGet_Vec_Info.argtypes = [ct.c_char_p]
        self.lib.ngGet_Vec_Info.restype = ct.POINTER(Vector)
        self.lib.ngSpice_CurPlot.restype = ct.c_char_p
        self.lib.ngSpice_AllVecs.argtypes = [ct.c_char_p]
        self.lib.ngSpice_AllVecs.restype = ct.POINTER(ct.c_char_p)

    def run(self, circuit):
        self.lib.ngSpice_Command(b'destroy all')
        self.log.clear()
        lines = [line.encode('ascii') for line in circuit.splitlines() if line.strip()]
        array = (ct.c_char_p * (len(lines)+1))(*lines,None)
        if self.lib.ngSpice_Circ(array) or self.lib.ngSpice_Command(b'run'):
            raise RuntimeError('\n'.join(self.log))
        plot = self.lib.ngSpice_CurPlot()
        if not plot or plot == b'const':
            raise RuntimeError('\n'.join(self.log))
        names = self.lib.ngSpice_AllVecs(plot)
        result = {}
        i = 0
        while names[i]:
            name = names[i]
            info = self.lib.ngGet_Vec_Info(plot+b'.'+name).contents
            if info.real:
                result[name.decode()] = np.ctypeslib.as_array(info.real,shape=(info.length,)).copy()
            i += 1
        if any('error' in line.lower() for line in self.log):
            raise RuntimeError('\n'.join(self.log))
        return result

    def release(self):
        """Release plots and every loaded circuit after copying result vectors."""
        self.lib.ngSpice_Command(b'destroy all')
        self.lib.ngSpice_Command(b'remcirc')
        # Shared ngspice also retains command history. The manual, section
        # 15.3.2.4 (ngSpice_Command), specifies NULL to free that allocation:
        # https://ngspice.sourceforge.io/docs/ngspice-44-manual.pdf
        self.lib.ngSpice_Command(None)
        self.log.clear()

_engine = None

def simulate(circuit):
    global _engine
    if _engine is None:
        _engine = Spice()
    return _engine.run(circuit)


def simulate_released(circuit):
    """Use the shared engine, freeing its circuit even when a run raises."""
    global _engine
    if _engine is None:
        _engine = Spice()
    try:
        return _engine.run(circuit)
    finally:
        _engine.release()
