import sys
import os
import serial
import serial.tools.list_ports
import time
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QIcon, QDoubleValidator
from PyQt5.QtCore import QTimer, QDateTime


APP_NAME = "PSW / PWR Controller"
APP_VERSION = "1.4"
APP_DATE = "Jan-12"

# ================== STATE DEFINITIONS ==================
STATE_IDLE = "IDLE"
STATE_RUN = "RUN"
STATE_NODETECT = "NO-DETECT"
STATE_ERROR = "ERROR"


# ============================================================
#  Resource helper
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
#  SERIAL CONTROLLER
# ============================================================
class PSWController:
    def __init__(self):
        self.ser = None
        self.profile = None
        self.last_idn = ""

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
        self.ser.write((cmd + "\n").encode("ascii"))

    def _readline(self):
        return self.ser.readline().decode(errors="ignore").strip()

    def query(self, cmd):
        self._write(cmd)
        return self._readline()

    def detect_model(self):
        idn = self.query("*IDN?")
        self.last_idn = idn
        idn_u = idn.upper()

        if "PSW" in idn_u:
            self.profile = PSWProfile()
        elif "PWR" in idn_u:
            self.profile = PWRProfile()
        else:
            self.profile = None  # NO-DETECT

    def set_vi(self, v, i):
        if self.profile:
            self.profile.set_vi(self, v, i)

    def output_on(self):
        if self.profile:
            self.profile.output_on(self)

    def output_off(self):
        if self.profile:
            self.profile.output_off(self)

    def measure_all(self):
        if self.profile:
            v, i = self.profile.measure_all(self)
            return v, i
        return None, None


# ============================================================
#  MAIN WINDOW
# ============================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(resource_path("subline.ui"), self)

        self.psw = PSWController()
        self.state = STATE_IDLE

        # Widgets
        self.combo_ports = self.findChild(QtWidgets.QComboBox, "comboBox")
        self.log_widget = self.findChild(QtWidgets.QTextEdit, "logg")
        self.lbl_model = self.findChild(QtWidgets.QLabel, "model_name")

        self.txtVoltage = self.findChild(QtWidgets.QLineEdit, "txtVoltage")
        self.txtCurrent = self.findChild(QtWidgets.QLineEdit, "txtCurrent")

        self.btn_connect = self.findChild(QtWidgets.QPushButton, "connect")
        self.btn_set = self.findChild(QtWidgets.QPushButton, "set")
        self.btn_on = self.findChild(QtWidgets.QPushButton, "on")
        self.btn_off = self.findChild(QtWidgets.QPushButton, "off")
        self.btn_read = self.findChild(QtWidgets.QPushButton, "read")

        self.btn_vs18 = self.findChild(QtWidgets.QPushButton, "vs18")
        self.btn_vs90 = self.findChild(QtWidgets.QPushButton, "vs90")
        self.btn_vs95 = self.findChild(QtWidgets.QPushButton, "vs95")

        self.lbl_18 = self.findChild(QtWidgets.QLabel, "status_18")
        self.lbl_90 = self.findChild(QtWidgets.QLabel, "status_90")
        self.lbl_95 = self.findChild(QtWidgets.QLabel, "status_95")

        # Blink
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.blink_run)
        self.active_label = None
        self.blink_state = False

        for lb in [self.lbl_18, self.lbl_90, self.lbl_95]:
            if lb:
                lb.setText("STOP")

        # Validators
        validator = QDoubleValidator(0.0, 1000.0, 3, self)
        self.txtVoltage.setValidator(validator)
        self.txtCurrent.setValidator(validator)

        # Signals
        self.btn_connect.clicked.connect(self.handle_connect)
        self.btn_set.clicked.connect(self.handle_set)
        self.btn_on.clicked.connect(self.handle_on)
        self.btn_off.clicked.connect(self.handle_off)
        self.btn_read.clicked.connect(self.handle_read)

        if self.btn_vs18:
            self.btn_vs18.clicked.connect(lambda: self.handle_vs(18.0, 10.0, self.lbl_18))
        if self.btn_vs90:
            self.btn_vs90.clicked.connect(lambda: self.handle_vs(21.0, 30.0, self.lbl_90))
        if self.btn_vs95:
            self.btn_vs95.clicked.connect(lambda: self.handle_vs(25.2, 45.0, self.lbl_95))

        self.populate_ports()
        self.set_state(STATE_IDLE)

    # ===================== STATE MACHINE =====================
    def set_state(self, new_state, reason=""):
        self.state = new_state
        self.log(f"STATE → {new_state} {reason}")

        if new_state in (STATE_IDLE, STATE_NODETECT, STATE_ERROR):
            self.reset_labels()

        enable = new_state == STATE_IDLE
        for b in [self.btn_set, self.btn_on, self.btn_read]:
            if b:
                b.setEnabled(enable)

        for b in [self.btn_vs18, self.btn_vs90, self.btn_vs95]:
            if b:
                b.setEnabled(enable)

    # ===================== HELPERS =====================
    def log(self, msg):
        ts = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_widget.append(f"[{ts}] {msg}")

    def populate_ports(self):
        self.combo_ports.clear()
        for p in serial.tools.list_ports.comports():
            self.combo_ports.addItem(p.device, p.device)

    def reset_labels(self):
        self.blink_timer.stop()
        self.active_label = None
        for lb in [self.lbl_18, self.lbl_90, self.lbl_95]:
            if lb:
                lb.setText("STOP")
                lb.setStyleSheet("")

    def blink_run(self):
        if not self.active_label:
            return
        self.blink_state = not self.blink_state
        if self.blink_state:
            self.active_label.setStyleSheet("background: yellow; color: black;")
        else:
            self.active_label.setStyleSheet("")

    # ===================== HANDLERS =====================
    def handle_connect(self):
        if self.psw.is_open():
            self.psw.close()
            self.lbl_model.setText("---")
            self.btn_connect.setText("Connect")
            self.set_state(STATE_IDLE, "(disconnect)")
            return

        port = self.combo_ports.currentData()
        try:
            self.psw.open(port)
            if self.psw.profile is None:
                self.lbl_model.setText("No detect")
                self.log(f"No detect: {self.psw.last_idn}")
                self.set_state(STATE_NODETECT)
            else:
                self.lbl_model.setText(self.psw.profile.name)
                self.log(f"Connected {port} ({self.psw.profile.name})")
                self.set_state(STATE_IDLE)

            self.btn_connect.setText("Disconnect")

        except Exception as e:
            self.log(str(e))
            self.set_state(STATE_ERROR)

    def handle_vs(self, v, i, label):
        if self.state != STATE_IDLE:
            self.log("Cannot start preset in current state")
            return

        try:
            self.psw.set_vi(v, i)
            self.psw.output_on()

            self.reset_labels()
            self.active_label = label
            label.setText("RUN")
            self.blink_timer.start(500)

            self.set_state(STATE_RUN, f"(VS {v}V)")

        except Exception as e:
            self.log(str(e))
            self.set_state(STATE_ERROR)

    def handle_set(self):
        if self.state != STATE_IDLE:
            return
        v = float(self.txtVoltage.text())
        i = float(self.txtCurrent.text())
        self.psw.set_vi(v, i)
        self.log(f"Set V={v} I={i}")

    def handle_on(self):
        if self.state != STATE_IDLE:
            return
        self.psw.output_on()
        self.set_state(STATE_RUN, "(manual ON)")

    def handle_off(self):
        self.psw.output_off()
        self.set_state(STATE_IDLE, "(OFF)")

    def handle_read(self):
        v, i = self.psw.measure_all()
        if v is not None:
            self.log(f"Read V={v:.3f} I={i:.3f}")


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
