import sys
import os
import serial
import serial.tools.list_ports
import time
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QIcon, QDoubleValidator
from PyQt5.QtCore import QTimer, QSettings, QDateTime
from PyQt5.QtWidgets import QMessageBox
from collections import deque

# Realtime plotting (optional)
try:
    import pyqtgraph as pg
except Exception:
    pg = None


APP_NAME = "PSW / PWR Controller"
APP_VERSION = "1.1"
APP_DATE = "Jan-12"


# ============================================================
#  PyInstaller resource helper
# ============================================================
def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS  # type: ignore
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)


# ============================================================
#  SCPI PROFILES
# ============================================================
class PSWProfile:
    name = "PSW"

    def set_vi(self, io, v, i):
        io._write(f"APPLy {v:.4f},{i:.4f}")

    def output_on(self, io):
        io._write("OUTPut:STATe ON")

    def output_off(self, io):
        io._write("OUTPut:STATe OFF")

    def measure_all(self, io):
        raw = io.query("MEASure:SCALar:ALL?")
        v, i = raw.split(",")
        return float(v), float(i)


class PWRProfile:
    name = "PWR-01"

    def set_vi(self, io, v, i):
        io._write(f"VOLT {v:.4f}")
        time.sleep(0.05)
        io._write(f"CURR {i:.4f}")

    def output_on(self, io):
        io._write("OUTP ON")

    def output_off(self, io):
        io._write("OUTP OFF")

    def measure_all(self, io):
        v = float(io.query("MEAS:VOLT?"))
        i = float(io.query("MEAS:CURR?"))
        return v, i


# ============================================================
#  SERIAL CONTROLLER (Auto detect model)
# ============================================================
class PSWController:
    def __init__(self):
        self.ser = None
        self.profile = None

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baudrate=9600, timeout=1):
        self.close()
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.detect_model()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.profile = None

    def _write(self, cmd):
        if not self.is_open():
            raise RuntimeError("Serial not open")
        self.ser.write((cmd + "\n").encode("ascii"))

    def _readline(self):
        return self.ser.readline().decode(errors="ignore").strip()

    def query(self, cmd):
        self._write(cmd)
        return self._readline()

    def detect_model(self):
        idn = self.query("*IDN?")
        idn_u = idn.upper()

        if "PSW" in idn_u:
            self.profile = PSWProfile()
        elif "PWR" in idn_u:
            self.profile = PWRProfile()
        else:
            raise RuntimeError(f"Unknown model: {idn}")

    # ----- Unified API -----
    def get_idn(self):
        return self.query("*IDN?")

    def set_vi(self, v, i):
        self.profile.set_vi(self, v, i)

    def output_on(self):
        self.profile.output_on(self)

    def output_off(self):
        self.profile.output_off(self)

    def measure_all(self):
        v, i = self.profile.measure_all(self)
        return v, i, f"{v},{i}"


# ============================================================
#  MAIN WINDOW
# ============================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        uic.loadUi(resource_path("subline.ui"), self)
        self.setWindowTitle(f"{APP_NAME} Ver {APP_VERSION}")
        self.setWindowIcon(QIcon(resource_path("psw.png")))

        self.psw = PSWController()

        self.actionInfor.triggered.connect(self.show_about_message)

        self.settings = QSettings("songhung.tr", "PSWController")
        last_port = self.settings.value("last_port", "", type=str)

        # Widgets
        self.combo_ports = self.findChild(QtWidgets.QComboBox, "comboBox")
        self.log_widget = self.findChild(QtWidgets.QTextEdit, "logg")

        self.lbl_model = self.findChild(QtWidgets.QLabel, "model_name")
        if self.lbl_model:
            self.lbl_model.setText("---")

        self.btn_connect = self.findChild(QtWidgets.QPushButton, "connect")
        self.btn_clean = self.findChild(QtWidgets.QPushButton, "clean")
        self.btn_test = self.findChild(QtWidgets.QPushButton, "test")
        self.btn_read = self.findChild(QtWidgets.QPushButton, "read")
        self.btn_refresh = self.findChild(QtWidgets.QPushButton, "refresh")

        self.txtVoltage = self.findChild(QtWidgets.QLineEdit, "txtVoltage")
        self.txtCurrent = self.findChild(QtWidgets.QLineEdit, "txtCurrent")
        self.btn_set = self.findChild(QtWidgets.QPushButton, "set")

        validator = QDoubleValidator(0.0, 1000.0, 3, self)
        self.txtVoltage.setValidator(validator)
        self.txtCurrent.setValidator(validator)

        self.btn_on = self.findChild(QtWidgets.QPushButton, "on")
        self.btn_off = self.findChild(QtWidgets.QPushButton, "off")

        # Signals
        self.btn_connect.clicked.connect(self.handle_connect)
        self.btn_clean.clicked.connect(lambda: self.log_widget.clear())
        self.btn_test.clicked.connect(self.handle_test)
        self.btn_read.clicked.connect(self.handle_read)
        self.btn_refresh.clicked.connect(self.handle_refresh)
        self.btn_set.clicked.connect(self.handle_set_v_i)
        self.btn_on.clicked.connect(self.handle_on)
        self.btn_off.clicked.connect(self.handle_off)

        self.populate_ports(last_port)
        self.set_controls_enabled(False)

    # --------------------
    def log(self, msg):
        ts = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_widget.append(f"[{ts}] {msg}")

    def set_controls_enabled(self, en):
        for w in [self.btn_test, self.btn_read, self.btn_set, self.btn_on, self.btn_off]:
            w.setEnabled(en)

    def populate_ports(self, preferred=None):
        self.combo_ports.clear()
        ports = list(serial.tools.list_ports.comports())
        sel = -1
        for i, p in enumerate(ports):
            self.combo_ports.addItem(p.device, p.device)
            if p.device == preferred:
                sel = i
        if sel >= 0:
            self.combo_ports.setCurrentIndex(sel)

    # --------------------
    def handle_refresh(self):
        self.populate_ports(self.combo_ports.currentData())
        self.log("Ports refreshed")

    def handle_connect(self):
        if self.psw.is_open():
            self.psw.close()
            self.log("Disconnected")
            self.btn_connect.setText("Connect")
            self.set_controls_enabled(False)
            if self.lbl_model:
                self.lbl_model.setText("---")
            return

        port = self.combo_ports.currentData()
        if not port:
            return

        try:
            self.psw.open(port)
            self.log(f"Connected {port}")
            self.log(f"Detected model: {self.psw.profile.name}")

            if self.lbl_model:
                self.lbl_model.setText(self.psw.profile.name)

            self.btn_connect.setText("Disconnect")
            self.set_controls_enabled(True)
            self.settings.setValue("last_port", port)

        except Exception as e:
            self.log(f"Error: {e}")
            if self.lbl_model:
                self.lbl_model.setText("ERR")

    def handle_test(self):
        try:
            self.log(self.psw.get_idn())
        except Exception as e:
            self.log(str(e))

    def handle_set_v_i(self):
        try:
            v = float(self.txtVoltage.text())
            i = float(self.txtCurrent.text())
            self.psw.set_vi(v, i)
            self.log(f"Set V={v}, I={i}")
        except Exception as e:
            self.log(str(e))

    def handle_on(self):
        try:
            self.psw.output_on()
            self.log("Output ON")
        except Exception as e:
            self.log(str(e))

    def handle_off(self):
        try:
            self.psw.output_off()
            self.log("Output OFF")
        except Exception as e:
            self.log(str(e))

    def handle_read(self):
        try:
            v, i, _ = self.psw.measure_all()
            self.log(f"Read: V={v:.3f} V, I={i:.3f} A")
        except Exception as e:
            self.log(str(e))

    def show_about_message(self):
        QMessageBox.information(
            self,
            "Infor",
            f"Ver {APP_VERSION}\n{APP_DATE}\nPIC. songhung.tr",
        )


# ============================================================
# MAIN
# ============================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
