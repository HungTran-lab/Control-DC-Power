import sys
import os
import re
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QIcon, QDoubleValidator
from PyQt5.QtCore import QTimer, QSettings, QDateTime, QThread, pyqtSignal
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
#  SCANNER THREAD (Serial @ 9600)
# ============================================================

class ScannerThread(QThread):
    """Đọc dữ liệu từ Scanner qua Serial ở background thread.

    Mỗi lần scanner gửi một dòng (thường kết thúc bằng \r\n), thread sẽ phát tín hiệu data_received(text).
    """
    data_received = pyqtSignal(str)
    status = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port: str, baudrate: int = 9600, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self._running = True
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.2)
            self.status.emit(f"Scanner connected {self.port} @ {self.baudrate}")
        except Exception as e:
            self.error.emit(f"Scanner open error: {e}")
            return

        while self._running:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                text = raw.decode(errors="ignore").strip()
                if text:
                    self.data_received.emit(text)
            except Exception as e:
                self.error.emit(f"Scanner read error: {e}")
                break

        # cleanup
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self.status.emit("Scanner disconnected")

    def stop(self):
        self._running = False
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

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
        last_scanner_port = self.settings.value("last_scanner_port", "", type=str)

        # Scanner auto-set debounce
        self._last_scan_text = ""
        self._last_scan_ms = 0


        # Widgets mapping
        self.combo_ports = self.findChild(QtWidgets.QComboBox, "comboBox")
        self.log_widget = self.findChild(QtWidgets.QTextEdit, "logg")

        self.btn_connect = self.findChild(QtWidgets.QPushButton, "connect")
        self.btn_clean = self.findChild(QtWidgets.QPushButton, "clean")
        self.btn_test = self.findChild(QtWidgets.QPushButton, "test")
        self.btn_read = self.findChild(QtWidgets.QPushButton, "read")
        self.btn_refresh = self.findChild(QtWidgets.QPushButton, "refresh")
        # ===== Scanner (Serial) =====
        # Prefer accessing widgets as attributes created by uic.loadUi (often more reliable than findChild).
        self.combo_scanner = getattr(self, "comboBox_2", None)
        self.btn_scanner_connect = getattr(self, "connect_2", None)
        self.lbl_scanner = getattr(self, "seri", None)

        # Fallback to findChild if attributes are not present (backward compatible with some UI loading styles).
        if self.combo_scanner is None:
            self.combo_scanner = self.findChild(QtWidgets.QComboBox, "comboBox_2")
        if self.btn_scanner_connect is None:
            self.btn_scanner_connect = self.findChild(QtWidgets.QPushButton, "connect_2")
        if self.lbl_scanner is None:
            self.lbl_scanner = self.findChild(QtWidgets.QLabel, "seri")

        self.scanner_thread = None
        self.scanner_ui_ok = (
            isinstance(self.combo_scanner, QtWidgets.QComboBox)
            and isinstance(self.btn_scanner_connect, QtWidgets.QPushButton)
            and isinstance(self.lbl_scanner, QtWidgets.QLabel)
        )

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
        if getattr(self, 'scanner_ui_ok', False):
            self.btn_scanner_connect.clicked.connect(self.handle_scanner_connect)
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
        if getattr(self, 'scanner_ui_ok', False):
            self.populate_scanner_ports(preferred_port=last_scanner_port)
        else:
            self.log("Scanner UI not found (comboBox_2/connect_2/seri) -> scanner disabled.")
        self.set_controls_enabled(False)

        # Init Connect button styles
        self.set_connect_button_state(self.btn_connect, self.psw.is_open())
        if getattr(self, 'scanner_ui_ok', False):
            self.set_connect_button_state(self.btn_scanner_connect, False)

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


    def set_connect_button_state(self, btn: QtWidgets.QPushButton, connected: bool):
        """Update Connect/Disconnect button UI.

        - When connected: text 'Disconnect' and green background.
        - When disconnected: text 'Connect' and no background color.
        """
        if btn is None:
            return
        if connected:
            btn.setText("Disconnect")
            btn.setStyleSheet("background-color: lightgreen;")
        else:
            btn.setText("Connect")
            btn.setStyleSheet("")

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


    def populate_scanner_ports(self, preferred_port=None):
        """Scan danh sách COM cho Scanner, nếu preferred_port còn tồn tại thì tự chọn nó."""
        if not getattr(self, 'scanner_ui_ok', False):
            return

        current_data = self.combo_scanner.currentData()
        self.combo_scanner.clear()

        index_to_select = -1
        ports = list(serial.tools.list_ports.comports())
        for idx, p in enumerate(ports):
            self.combo_scanner.addItem(p.device, p.device)
            if preferred_port and p.device == preferred_port:
                index_to_select = idx
            elif not preferred_port and current_data and p.device == current_data:
                index_to_select = idx

        if index_to_select >= 0:
            self.combo_scanner.setCurrentIndex(index_to_select)

    def handle_scanner_connect(self):
        """Connect/Disconnect Scanner (Serial @ 9600) bằng thread."""
        if not getattr(self, 'scanner_ui_ok', False):
            self.log("Scanner UI not available.")
            return

        # Nếu đang chạy -> bấm lần nữa là Disconnect
        if self.scanner_thread and self.scanner_thread.isRunning():
            self.stop_scanner()
            return

        port = self.combo_scanner.currentData()
        if not port:
            self.log("No Scanner COM selected")
            return

        self.lbl_scanner.setText("")
        self.scanner_thread = ScannerThread(port=port, baudrate=9600, parent=self)
        self.scanner_thread.data_received.connect(self.on_scanner_data)
        self.scanner_thread.status.connect(self.on_scanner_status)
        self.scanner_thread.error.connect(self.on_scanner_error)

        self.scanner_thread.start()
        self.set_connect_button_state(self.btn_scanner_connect, True)

        # Lưu COM vừa connect thành công
        self.settings.setValue("last_scanner_port", port)
    def parse_scanner_to_vi(self, text: str):
        """Parse scanner text into (V, I).

        Supported examples:
          - "18,10"  "18;10"  "18 10"
          - "V=18 I=10"  "V18 I10"  "18V 10A"
          - Presets: "VS18", "VS21", "VS25"
          - Output: "ON"  "OFF"
        Returns:
          - ("PRESET", 18/21/25) for preset codes
          - ("OUTPUT", True/False) for output on/off
          - (v, i) floats for numeric pairs
          - (v, None) if only voltage can be parsed
          - None if not recognized
        """
        if not text:
            return None

        t = text.strip().upper()

        # Output control (scan: ON / OFF)
        if re.fullmatch(r"(?:ON|OUTPUT\s*ON|OUT\s*ON)$", t):
            return ("OUTPUT", True)
        if re.fullmatch(r"(?:OFF|OUTPUT\s*OFF|OUT\s*OFF)$", t):
            return ("OUTPUT", False)

        # Preset keywords
        if "VS18" in t:
            return ("PRESET", 18)
        if "VS21" in t:
            return ("PRESET", 21)
        if "VS25" in t:
            return ("PRESET", 25)

        # Try explicit V and I tokens
        m = re.search(r"V\s*[:=]?\s*([0-9]+(?:\.[0-9]+)?)", t)
        n = re.search(r"I\s*[:=]?\s*([0-9]+(?:\.[0-9]+)?)", t)
        if m and n:
            return (float(m.group(1)), float(n.group(1)))

        # Alternative tokens
        m = re.search(r"(?:VOLT|V)\s*[:=]?\s*([0-9]+(?:\.[0-9]+)?)", t)
        n = re.search(r"(?:CURR|AMP|A|I)\s*[:=]?\s*([0-9]+(?:\.[0-9]+)?)", t)
        if m and n:
            return (float(m.group(1)), float(n.group(1)))

        # Generic numbers
        nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", t)
        if len(nums) >= 2:
            try:
                return (float(nums[0]), float(nums[1]))
            except Exception:
                return None
        if len(nums) == 1:
            try:
                return (float(nums[0]), None)
            except Exception:
                return None
        return None

    def apply_scanner_vi(self, text: str):
        """Use scanner data to set V/I (Opt2)."""
        # Debounce duplicate scans (some scanners emit twice)
        now_ms = QDateTime.currentMSecsSinceEpoch()
        if text == self._last_scan_text and (now_ms - self._last_scan_ms) < 500:
            return
        self._last_scan_text = text
        self._last_scan_ms = now_ms

        parsed = self.parse_scanner_to_vi(text)
        if parsed is None:
            return  # ignore unknown format silently

        # Handle output ON/OFF codes
        if isinstance(parsed, tuple) and len(parsed) == 2 and parsed[0] == "OUTPUT":
            want_on = bool(parsed[1])
            if not self.psw.is_open():
                self.log(f"Scanner {'ON' if want_on else 'OFF'} received, but PWR not connected")
                return
            if want_on:
                self.handle_on()
            else:
                self.handle_off()
            return

        # Handle preset codes
        if isinstance(parsed, tuple) and len(parsed) == 2 and parsed[0] == "PRESET":
            preset = parsed[1]
            if not self.psw.is_open():
                self.log(f"Scanner preset {preset} received, but PWR not connected")
                return
            self.set_stick(int(preset))
            return

        # Numeric V/I
        v, i = parsed

        # If only voltage provided, reuse current from txtCurrent or last target_i
        if i is None:
            i2 = None
            try:
                if self.txtCurrent and self.txtCurrent.text().strip():
                    i2 = float(self.txtCurrent.text())
            except Exception:
                i2 = None
            if i2 is None:
                try:
                    i2 = float(getattr(self.psw, "target_i", 10.0))
                except Exception:
                    i2 = 10.0
            i = i2

        # Update UI fields to reflect scanned values
        try:
            if self.txtVoltage is not None:
                self.txtVoltage.setText(f"{float(v):.3f}")
            if self.txtCurrent is not None:
                self.txtCurrent.setText(f"{float(i):.3f}")
        except Exception:
            pass

        if not self.psw.is_open():
            self.log(f"Scanner V/I received (V={v}, I={i}) but PWR not connected")
            return

        try:
            self.psw.set_vi(float(v), float(i))
        except Exception as e:
            self.handle_connection_lost(e)
            return

        self.log(f"Scanner set V={float(v):.3f}, I={float(i):.3f}")

    def on_scanner_status(self, msg: str):
        self.log(msg)

    def on_scanner_data(self, text: str):
        """Hiển thị data scan được lên label 'seri' và dùng nó để set V/I (Opt2)."""
        if getattr(self, 'lbl_scanner', None):
            self.lbl_scanner.setText(text)
        try:
            self.apply_scanner_vi(text)
        except Exception as e:
            # never crash UI on parse errors
            self.log(f"Scanner parse/apply error: {e}")

    def on_scanner_error(self, msg: str):
        self.log(msg)
        self.stop_scanner()

    def stop_scanner(self):
        if self.scanner_thread:
            try:
                self.scanner_thread.stop()
            except Exception:
                pass
            # đợi thread dừng (ngắn)
            try:
                self.scanner_thread.wait(500)
            except Exception:
                pass

        if getattr(self, 'btn_scanner_connect', None):
            self.set_connect_button_state(self.btn_scanner_connect, False)

    def handle_refresh(self):
        """Nút Refresh: scan lại COM, cố gắng giữ COM đang chọn."""
        preferred = self.combo_ports.currentData()
        self.populate_ports(preferred_port=preferred)
        count_pwr = self.combo_ports.count()

        if getattr(self, 'scanner_ui_ok', False):
            preferred_sc = self.combo_scanner.currentData()
            self.populate_scanner_ports(preferred_port=preferred_sc)
            count_sc = self.combo_scanner.count()
            self.log(f"Refreshed Done: PWR={count_pwr} port(s), Scanner={count_sc} port(s)")
        else:
            self.log(f"Refreshed Done:{count_pwr} port(s) found")

    def handle_connection_lost(self, err):
        """Gọi hàm này khi có lỗi giao tiếp -> tự chuyển về trạng thái Disconnected."""
        self.log(f"Connection lost: {err}")
        self.stop_streaming()
        self.psw.close()
        self.set_connect_button_state(self.btn_connect, False)
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
            self.set_connect_button_state(self.btn_connect, False)
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
            self.set_connect_button_state(self.btn_connect, True)
            self.set_controls_enabled(True)

            # Lưu COM vừa connect thành công
            self.settings.setValue("last_port", port)
        except Exception as e:
            self.log(f"Error: {e}")
            self.set_controls_enabled(False)
            self.set_connect_button_state(self.btn_connect, False)

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


    def closeEvent(self, event):
        """Đảm bảo đóng Serial và dừng thread khi thoát."""
        try:
            self.stop_streaming()
        except Exception:
            pass
        try:
            self.stop_scanner()
        except Exception:
            pass
        try:
            if self.psw and self.psw.is_open():
                self.psw.close()
        except Exception:
            pass
        event.accept()

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