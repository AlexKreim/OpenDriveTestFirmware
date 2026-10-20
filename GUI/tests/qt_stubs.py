"""Minimal stand-ins for PySide6 / pyqtgraph so the GUI logic can be exercised without Qt (CI-style smoke test).
They implement just enough behaviour (values, ranges, signals, combo boxes) for main.py's logic."""
import sys
import types

SIGNALS = {"clicked", "valueChanged", "toggled", "timeout", "currentIndexChanged", "activated", "returnPressed"}


class _Sig:
    def __init__(self):
        self.slots = []

    def connect(self, f):
        self.slots.append(f)

    def emit(self, *a):
        for f in list(self.slots):
            f(*a)


def _noop(*a, **k):
    return None


class Base:
    def __init__(self, *a, **k):
        self._sigs = {}
        self._blocked = False
        self._enabled = True
        self._text = a[0] if a and isinstance(a[0], str) else ""
        self._value = 0
        self._lo, self._hi = 0, 99
        self._checked = False
        self._items = []
        self._idx = -1
        self._lines = []
        self._visible = True
        if "activated" in k:
            self._sigs["activated"] = _Sig()
            self._sigs["activated"].connect(k["activated"])

    def __getattr__(self, n):
        if n.startswith("__") or n in ("_sigs",):
            raise AttributeError(n)
        if n in SIGNALS:
            return self._sigs.setdefault(n, _Sig())
        return _noop

    def _emit(self, n, *a):
        if not self._blocked:
            self._sigs.setdefault(n, _Sig()).emit(*a)

    # generic
    def lineEdit(self):
        return self

    def blockSignals(self, b):
        self._blocked = b

    def setEnabled(self, b):
        self._enabled = bool(b)

    def isEnabled(self):
        return self._enabled

    def setText(self, t):
        self._text = t

    def text(self):
        return self._text

    def statusBar(self):
        return Base()

    def font(self):
        return Base()

    # numeric widgets
    def setRange(self, lo, hi):
        self._lo, self._hi = lo, hi

    def setMaximum(self, hi):
        self._hi = hi

    def setValue(self, v):
        v = min(max(v, self._lo), self._hi)
        if v != self._value:
            self._value = v
            self._emit("valueChanged", v)

    def value(self):
        return self._value

    # checkable
    def setChecked(self, b):
        b = bool(b)
        if b != self._checked:
            self._checked = b
            self._emit("toggled", b)

    def isChecked(self):
        return self._checked

    # combo
    def addItem(self, text, data=None):
        self._items.append((text, data))
        if self._idx < 0:
            self._idx = 0

    def clear(self):
        self._items = []
        self._idx = -1

    def currentText(self):
        return self._items[self._idx][0] if 0 <= self._idx < len(self._items) else ""

    def currentData(self):
        return self._items[self._idx][1] if 0 <= self._idx < len(self._items) else None

    def currentIndex(self):
        return self._idx

    def setCurrentIndex(self, i):
        if i != self._idx:
            self._idx = i
            self._emit("currentIndexChanged", i)

    def findText(self, t):
        for i, (x, _) in enumerate(self._items):
            if x == t:
                return i
        return -1

    # plain text edit
    def appendPlainText(self, t):
        self._lines.append(t)

    # plots
    def plot(self, *a, **k):
        return Curve()

    def getAxis(self, *a):
        return Base()

    def setYRange(self, lo, hi, **k):
        self.yrange = (lo, hi)

    def setLabel(self, side, text=None, **k):
        self.__dict__.setdefault('labels', {})[side] = (text, k.get('units'))

    # timers
    def start(self, *a):
        self._active = True

    def stop(self):
        self._active = False


class TabWidget(Base):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self._tabs = []
        self._cur = 0

    def addTab(self, w, label):
        self._tabs.append(w)

    def widget(self, i):
        return self._tabs[i]

    def currentWidget(self):
        return self._tabs[self._cur]

    def setCurrentWidget(self, w):
        self._cur = self._tabs.index(w)

    def currentIndex(self):
        return self._cur


class Curve(Base):
    def setData(self, x, y):
        self.x, self.y = x, y


class MessageBox(Base):
    calls = []

    class StandardButton:
        Yes = 1
        No = 2

    @staticmethod
    def _rec(kind, args):
        MessageBox.calls.append((kind, args[2] if len(args) > 2 else ""))
        return MessageBox.StandardButton.Yes

    @staticmethod
    def warning(*a):
        return MessageBox._rec("warning", a)

    @staticmethod
    def critical(*a):
        return MessageBox._rec("critical", a)

    @staticmethod
    def information(*a):
        return MessageBox._rec("information", a)

    @staticmethod
    def question(*a):
        return MessageBox._rec("question", a)


class InputDialog(Base):
    next_text = ("", False)

    @staticmethod
    def getText(*a, **k):
        return InputDialog.next_text


class FileDialog(Base):
    next_path = ""

    @staticmethod
    def getSaveFileName(*a, **k):
        return (FileDialog.next_path, "")


def install():
    widgets = types.ModuleType("PySide6.QtWidgets")
    for n in ("QMainWindow", "QWidget", "QVBoxLayout", "QHBoxLayout", "QGridLayout", "QFormLayout", "QSplitter",
              "QGroupBox", "QScrollArea", "QFrame", "QLabel", "QPushButton", "QComboBox", "QDoubleSpinBox", "QSpinBox", "QSlider",
              "QDial", "QRadioButton", "QCheckBox", "QPlainTextEdit", "QApplication"):
        setattr(widgets, n, type(n, (Base,), {}))
    widgets.QTabWidget = TabWidget
    widgets.QFrame.Shape = type("Shape", (), {"NoFrame": 0})
    widgets.QComboBox.InsertPolicy = type("InsertPolicy", (), {"NoInsert": 0})
    widgets.QMessageBox = MessageBox
    widgets.QInputDialog = InputDialog
    widgets.QFileDialog = FileDialog

    core = types.ModuleType("PySide6.QtCore")
    core.QTimer = type("QTimer", (Base,), {})

    class Qt:
        class Orientation:
            Horizontal = 1
            Vertical = 2
    core.Qt = Qt

    gui = types.ModuleType("PySide6.QtGui")
    gui.QShortcut = type("QShortcut", (Base,), {})
    gui.QKeySequence = type("QKeySequence", (Base,), {})
    gui.QFont = type("QFont", (Base,), {"StyleHint": type("StyleHint", (), {"Monospace": 0})})

    pyside = types.ModuleType("PySide6")
    pyside.QtWidgets, pyside.QtCore, pyside.QtGui = widgets, core, gui

    pg = types.ModuleType("pyqtgraph")
    pg.PlotWidget = type("PlotWidget", (Base,), {})
    pg.mkPen = lambda *a, **k: None
    pg.setConfigOptions = lambda **k: None

    sys.modules.update({"PySide6": pyside, "PySide6.QtWidgets": widgets, "PySide6.QtCore": core,
                        "PySide6.QtGui": gui, "pyqtgraph": pg})
