import sys
import os
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QIcon, QDoubleValidator
from PyQt5.QtCore import QTimer, QSettings, QDateTime
from collections import deque

# Realtime plotting (optional)
try:
    import pyqtgraph as pg
except Exception:
    pg = None


from PyQt5.QtWidgets import QMessageBox

APP_NAME = "PWR Controller"
APP_VERSION = "1.0"
APP_DATE = "Jan-14"


# ============================================================
#  Hỗ trợ load file .ui / icon khi đóng gói PyInstaller
# ============================================================
def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS  # type: ignore
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)


# ============================================================
#  Lớp điều khiển PSW qua Serial / SCPI
# ============================================================
class PSWController:
    def __init__(self):
        self.ser = None
        self.target_v = 0.0
        self.target_i = 0.0

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baudrate=9600, timeout=1):
        self.close()
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def close(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _write(self, cmd):
        if not self.is_open():
            raise RuntimeError("Serial not open")
        if not cmd.endswith("\n"):
            cmd = cmd + "\n"
        try:
            self.ser.write(cmd.encode("ascii"))
        except Exception as e:
            self.close()
            raise RuntimeError(f"Serial write error: {e}")

    def _readline(self):
        if not self.is_open():
            raise RuntimeError("Serial not open")
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            return line
        except Exception as e:
            self.close()
            raise RuntimeError(f"Serial read error: {e}")

    def query(self, cmd):
        self._write(cmd)
        return self._readline()

    def get_idn(self):
        return self.query("*IDN?")

    def set_vi(self, v, i):
        self.target_v = v
        self.target_i = i
        # set_V: VOLT
        self._write(f"VOLT {v:.4f}")
        # SET_I: CURR
        self._write(f"CURR {i:.4f}")

    def set_voltage(self, v):
        # set_V: VOLT
        self.target_v = v
        self._write(f"VOLT {v:.4f}")

    def set_current(self, i):
        # SET_I: CURR
        self.target_i = i
        self._write(f"CURR {i:.4f}")


    def output_on(self):
        self._write("OUTP ON")

    def output_off(self):
        self._write("OUTP OFF")

    def measure_all(self):
        raw_v = self.query("MEAS:VOLT?")
        raw_i = self.query("MEAS:CURR?")

        try:
            v = float(raw_v.strip())
        except Exception:
            v = None

        try:
            i = float(raw_i.strip())
        except Exception:
            i = None

        raw = f"{raw_v.strip()},{raw_i.strip()}"
        return v, i, raw



# ============================================================
#  MAIN WINDOW
# ============================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Load UI từ file .ui
        uic.loadUi(resource_path("subline.ui"), self)
        self.setWindowTitle(f"{APP_NAME} Ver {APP_VERSION}")
        self.setWindowIcon(QIcon(resource_path("psw.png")))

        self.psw = PSWController()

        # action About (menu Infor)
        self.actionInfor.triggered.connect(self.show_about_message)

        # Cố định kích thước cửa sổ
        # self.setFixedSize(771, 609)

        # QSettings: nhớ COM lần cuối
        self.settings = QSettings("songhung.tr", "PSWController")
        last_port = self.settings.value("last_port", "", type=str)

        # Widgets mapping
        self.combo_ports = self.findChild(QtWidgets.QComboBox, "comboBox")
        self.log_widget = self.findChild(QtWidgets.QTextEdit, "logg")

        self.btn_connect = self.findChild(QtWidgets.QPushButton, "connect")
        self.btn_clean = self.findChild(QtWidgets.QPushButton, "clean")
        self.btn_test = self.findChild(QtWidgets.QPushButton, "test")
        self.btn_read = self.findChild(QtWidgets.QPushButton, "read")
        self.btn_refresh = self.findChild(QtWidgets.QPushButton, "refresh")

        self.txtVoltage = self.findChild(QtWidgets.QLineEdit, "txtVoltage")
        self.txtCurrent = self.findChild(QtWidgets.QLineEdit, "txtCurrent")
        self.btn_set = self.findChild(QtWidgets.QPushButton, "set")

        # Validator cho Voltage / Current (số thực, 0..1000, 3 chữ số thập phân)
        validator = QDoubleValidator(0.0, 1000.0, 3, self)
        validator.setNotation(QDoubleValidator.StandardNotation)
        self.txtVoltage.setValidator(validator)
        self.txtCurrent.setValidator(validator)

        self.btn_on = self.findChild(QtWidgets.QPushButton, "on")
        self.btn_off = self.findChild(QtWidgets.QPushButton, "off")

        self.vs18 = self.findChild(QtWidgets.QPushButton, "vs18")
        self.vs21 = self.findChild(QtWidgets.QPushButton, "vs21")
        self.vs25 = self.findChild(QtWidgets.QPushButton, "vs25")

        self.status_18 = self.findChild(QtWidgets.QLabel, "status_18")
        self.status_21 = self.findChild(QtWidgets.QLabel, "status_21")
        self.status_25 = self.findChild(QtWidgets.QLabel, "status_25")

        # Timer for blinking RUN
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.blink_run)
        self.blink_state = False
        self.active_label = None

        # ====================================================
        #  STREAM PLOT (0.1s) - requires:
        #   - QRadioButton objectName: radioButton_stream
        #   - QWidget (placeholder) objectName: widget_plot
        #   - Optional dependency: pyqtgraph (pip install pyqtgraph)
        # ====================================================
        self.radio_stream = self.findChild(QtWidgets.QRadioButton, "radioButton_stream")
        self.plot_host = self.findChild(QtWidgets.QWidget, "widget_plot")

        self.stream_timer = QTimer()
        self.stream_timer.setInterval(100)  # 0.1 s
        self.stream_timer.timeout.connect(self.stream_sample)

        self.t0_ms = None
        self.stream_window_s = 5.0  # reset plot every 5 seconds to keep GUI light
        self.buf_t = deque(maxlen=60)  # ~6s @ 10Hz
        self.buf_i = deque(maxlen=60)

        self.plot_widget = None
        self.curve_v = None
        self.curve_i = None
        self._init_stream_plot()

        # SIGNALS
        self.btn_connect.clicked.connect(self.handle_connect)
        self.btn_clean.clicked.connect(lambda: self.log_widget.clear())
        self.btn_test.clicked.connect(self.handle_test)
        self.btn_read.clicked.connect(self.handle_read)
        self.btn_refresh.clicked.connect(self.handle_refresh)
        self.btn_set.clicked.connect(self.handle_set_v_i)

        self.btn_on.clicked.connect(self.handle_on)
        self.btn_off.clicked.connect(self.handle_off)

        self.vs18.clicked.connect(lambda: self.set_stick(18))
        self.vs21.clicked.connect(lambda: self.set_stick(21))
        self.vs25.clicked.connect(lambda: self.set_stick(25))
        

        if self.radio_stream:
            self.radio_stream.toggled.connect(self.on_stream_toggled)

        # Scan COM lần đầu, ưu tiên chọn lại COM đã lưu
        self.populate_ports(preferred_port=last_port)
        self.set_controls_enabled(False)

    # ----- Helpers -----
    def log(self, msg):
        ts = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_widget.append(f"[{ts}] {msg}")

    def set_controls_enabled(self, enabled):
        for w in [
            self.btn_test,
            self.btn_read,
            self.btn_set,
            self.btn_on,
            self.btn_off,
            self.vs18,
            self.vs21,
            self.vs25,
        ]:
            w.setEnabled(enabled)

    def populate_ports(self, preferred_port=None):
        """Scan danh sách COM, nếu preferred_port còn tồn tại thì tự chọn nó."""
        current_data = self.combo_ports.currentData()
        self.combo_ports.clear()

        index_to_select = -1
        ports = list(serial.tools.list_ports.comports())
        for idx, p in enumerate(ports):
            self.combo_ports.addItem(p.device, p.device)
            if preferred_port and p.device == preferred_port:
                index_to_select = idx
            elif not preferred_port and current_data and p.device == current_data:
                index_to_select = idx

        if index_to_select >= 0:
            self.combo_ports.setCurrentIndex(index_to_select)

    def handle_refresh(self):
        """Nút Refresh: scan lại COM, cố gắng giữ COM đang chọn."""
        preferred = self.combo_ports.currentData()
        self.populate_ports(preferred_port=preferred)

        count = self.combo_ports.count()
        self.log(f"Refreshed Done:{count} port(s) found")

    def handle_connection_lost(self, err):
        """Gọi hàm này khi có lỗi giao tiếp -> tự chuyển về trạng thái Disconnected."""
        self.log(f"Connection lost: {err}")
        self.stop_streaming()
        self.psw.close()
        self.btn_connect.setText("Connect")
        self.set_controls_enabled(False)
        self.reset_output_ui()

    def reset_output_ui(self):
        """Reset UI về trạng thái output OFF / AUTO dừng."""
        self.blink_timer.stop()
        self.blink_state = False
        self.active_label = None

        # self.lbl_70.setText("STOP")
        # self.lbl_25.setText("STOP")
        self.status_18.setStyleSheet("")
        self.status_21.setStyleSheet("")
        self.status_25.setStyleSheet("")

        self.btn_on.setStyleSheet("")
        self.btn_off.setStyleSheet("background-color: lightgray;")


    # ----- Streaming plot (every 0.1s) -----
    def _init_stream_plot(self):
        """Init plot widget inside widget_plot. Safe to call even if UI parts are missing."""
        # If user doesn't have these objects in .ui -> just skip
        if self.radio_stream is None or self.plot_host is None:
            return

        if pg is None:
            self.log("Plot disabled: missing dependency 'pyqtgraph' (pip install pyqtgraph)")
            self.radio_stream.setEnabled(False)
            return

        # If widget_plot is already a promoted pg.PlotWidget, use it directly.
        if isinstance(self.plot_host, pg.PlotWidget):
            self.plot_widget = self.plot_host
        else:
            self.plot_widget = pg.PlotWidget()
            lay = self.plot_host.layout()
            if lay is None:
                lay = QtWidgets.QVBoxLayout(self.plot_host)
                lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(self.plot_widget)

        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.setLabel("left", "Current", units="A")

        # One curve: Current (A)
        self.curve_v = None
        self.curve_i = self.plot_widget.plot([], [], name="I (A)")
        # Fix X range to 0..window
        self.plot_widget.setXRange(0, self.stream_window_s, padding=0)
        self.plot_widget.enableAutoRange(axis='y', enable=True)

    def on_stream_toggled(self, checked: bool):
        """RadioButton toggled -> start/stop streaming."""
        if checked:
            if not self.psw.is_open():
                self.log("Not connected -> cannot start streaming")
                # revert UI state safely
                self.radio_stream.blockSignals(True)
                self.radio_stream.setChecked(False)
                self.radio_stream.blockSignals(False)
                return
            self.start_streaming()
        else:
            self.stop_streaming()

    def start_streaming(self):
        if pg is None or self.plot_widget is None:
            return
        self.buf_t.clear()
        self.buf_i.clear()
        self.t0_ms = QDateTime.currentMSecsSinceEpoch()
        self.stream_timer.start()
        self.log("Streaming ON (100 ms)")

    def stop_streaming(self):
        if hasattr(self, "stream_timer") and self.stream_timer.isActive():
            self.stream_timer.stop()
            self.log("Streaming OFF")

    def stream_sample(self):
        """Timer tick: read V/I and update plot."""
        if not self.psw.is_open():
            self.stop_streaming()
            if self.radio_stream:
                self.radio_stream.blockSignals(True)
                self.radio_stream.setChecked(False)
                self.radio_stream.blockSignals(False)
            return

        try:
            v, i, _ = self.psw.measure_all()
        except Exception as e:
            self.handle_connection_lost(e)
            if self.radio_stream:
                self.radio_stream.blockSignals(True)
                self.radio_stream.setChecked(False)
                self.radio_stream.blockSignals(False)
            return

        if v is None or i is None or self.t0_ms is None:
            return

        now_ms = QDateTime.currentMSecsSinceEpoch()
        t = (now_ms - self.t0_ms) / 1000.0

        # Reset plot every N seconds to avoid overloading the GUI
        if t >= self.stream_window_s:
            self.t0_ms = now_ms
            self.buf_t.clear()
            self.buf_i.clear()
            t = 0.0
            if self.plot_widget is not None:
                self.plot_widget.setXRange(0, self.stream_window_s, padding=0)

        self.buf_t.append(t)
        self.buf_i.append(i)

        # Update curve (Current only)
        if self.curve_i is not None:
            self.curve_i.setData(list(self.buf_t), list(self.buf_i))

    # ----- Handlers -----
    def handle_connect(self):
        # Nếu đang mở -> bấm lần nữa là Disconnect
        if self.psw.is_open():
            self.psw.close()
            self.log("Disconnected.")
            self.btn_connect.setText("Connect")
            self.set_controls_enabled(False)
            self.stop_streaming()
            return

        port = self.combo_ports.currentData()
        if not port:
            self.log("No COM selected")
            return

        try:
            self.psw.open(port)
            self.log(f"Connected {port}")
            self.btn_connect.setText("Disconnect")
            self.set_controls_enabled(True)

            # Lưu COM vừa connect thành công
            self.settings.setValue("last_port", port)
        except Exception as e:
            self.log(f"Error: {e}")
            self.set_controls_enabled(False)

    def handle_test(self):
        if not self.psw.is_open():
            self.log("Not connected")
            return
        try:
            self.log(self.psw.get_idn())
        except Exception as e:
            self.handle_connection_lost(e)

    def handle_read(self):
        if not self.psw.is_open():
            self.log("Not connected")
            return
        try:
            v, i, raw = self.psw.measure_all()
        except Exception as e:
            self.handle_connection_lost(e)
            return

        if v is None:
            self.log(f"Read error: {raw}")
        else:
            self.log(f"Read: V={v:.3f} V, I={i:.3f} A  (raw: {raw})")

    def handle_set_v_i(self):
        if not self.psw.is_open():
            self.log("Not connected")
            return

        try:
            v = float(self.txtVoltage.text())
            a = float(self.txtCurrent.text())
        except Exception:
            self.log("Invalid V/I")
            return

        try:
            self.psw.set_vi(v, a)
        except Exception as e:
            self.handle_connection_lost(e)
            return

        self.log(f"Set V={v}, I={a}")

    def handle_on(self):
        if not self.psw.is_open():
            self.log("Not connected")
            return

        try:
            self.psw.output_on()
        except Exception as e:
            self.handle_connection_lost(e)
            return

        self.log("Output ON")
        self.btn_on.setStyleSheet("background-color: red; color: white;")
        self.btn_off.setStyleSheet("")

    def handle_off(self):
        if not self.psw.is_open():
            self.log("Not connected")
            return

        try:
            self.psw.output_off()
        except Exception as e:
            self.handle_connection_lost(e)
            return

        self.log("Output OFF")
        self.reset_output_ui()
        self.log("AUTO ON disabled")

    # ----- Stick mode VS70 / VS25 -----
    def set_stick(self, value):
        if not self.psw.is_open():
            self.log("Not connected")
            return

        # Stop previous blinking
        self.blink_timer.stop()
        self.status_18.setStyleSheet("")
        self.status_21.setStyleSheet("")
        self.status_25.setStyleSheet("")
        self.active_label = None

        try:
            if value == 18:
                self.psw.set_vi(18.0, 10.0)
                self.log("VS70: Set 18V / 10A")

                self.psw.output_on()
                self.log("Output AUTO ON")

                self.active_label = self.status_18
                self.status_18.setText("RUN")
                self.status_21.setText("...")
                self.status_25.setText("...")

            elif value == 21:
                self.psw.set_vi(21.0, 10.0)
                self.log("VS25: Set 25V / 10A")

                self.psw.output_on()
                self.log("Output AUTO ON")

                self.active_label = self.status_21
                self.status_21.setText("RUN")
                self.status_18.setText("...")
                self.status_25.setText("...")
            elif value == 25:
                self.psw.set_vi(25.2, 10.0)
                self.log("VS25: Set 25V / 10A")

                self.psw.output_on()
                self.log("Output AUTO ON")

                self.active_label = self.status_25
                self.status_25.setText("RUN")
                self.status_18.setText("...")
                self.status_21.setText("...")
        except Exception as e:
            self.handle_connection_lost(e)
            return

        # Start blinking
        if self.active_label:
            self.blink_state = False
            self.blink_timer.start(500)  # 500 ms

    # ----- Blink handler -----
    def blink_run(self):
        if not self.active_label:
            return

        self.blink_state = not self.blink_state
        if self.blink_state:
            self.active_label.setStyleSheet("background-color: yellow; color: black;")
        else:
            self.active_label.setStyleSheet("")

    def show_about_message(self):
        QMessageBox.information(
            self,
            "Infor",
            f"Ver {APP_VERSION}.\n{APP_DATE}\nPIC. songhung.tr",
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
