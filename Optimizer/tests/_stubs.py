"""Stand-ins for the Windows-only and hardware modules the optimizer imports.

Importing this module before the optimizer modules lets the tests run on any OS.
A real module is used when it is installed.
"""
import ctypes
import importlib
import sys
from unittest import mock

for _name in ('win32gui', 'win32ui', 'cv2', 'pytesseract', 'bleak', 'bleak.exc',
              'PyQt5', 'PyQt5.QtWidgets', 'PyQt5.QtCore', 'PyQt5.QtGui', 'pyqtgraph'):
    if _name in sys.modules:
        continue
    try:
        importlib.import_module(_name)
    except ImportError:
        sys.modules[_name] = mock.MagicMock(name=_name)

if not hasattr(ctypes, 'windll'):
    ctypes.windll = mock.MagicMock(name='windll')

if isinstance(sys.modules['bleak.exc'], mock.MagicMock):
    # Caught in an except clause, so it must be an exception class, as in bleak
    sys.modules['bleak.exc'].BleakError = type('BleakError', (Exception,), {})
