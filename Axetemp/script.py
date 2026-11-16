import sys
import time
import re
import threading
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

import serial
import serial.tools.list_ports

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt


class BMP280ControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 + BMP280 Control & Monitor System")
        self.root.geometry("1100x780")
        self.root.minsize(950, 650)


        # Змінні зв'язку
        self.ser = None
        self.is_running = False
        self.read_thread = None

        # Буфери для збереження останніх 60 вимірювань графіків
        self.max_points = 60
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.press_data = deque(maxlen=self.max_points)
        self.start_time = None

        # Створення UI
        self._setup_styles()
        self._build_ui()

        # При закритті вікна вірно зупиняємо потік
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _setup_styles(self):
        self.style = ttk.Style()
        self.style.theme_use("clam")

        # Загальні налаштування стилів
        self.style.configure("TLabelframe", padding=8, relief="groove")
        self.style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"), foreground="#1a365d")
        self.style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=4)
        self.style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))

    def _build_ui(self):
        # Головна контейнерна сітка
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Ліва панель (Керування + Налаштування + Термінал)
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 10))

        # Права панель (Цифрові індикатори + Графіки)
        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ==================== ЛІВА ПАНЕЛЬ ====================

        # 1. Блок підключення COM-порту
        conn_frame = ttk.LabelFrame(left_panel, text=" З'єднання UART ")
        conn_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(conn_frame, text="Порт:").grid(row=0, column=0, sticky=tk.W, padx=2)
        self.port_combo = ttk.Combobox(conn_frame, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4)

        btn_refresh = ttk.Button(conn_frame, text="🔄", width=3, command=self.refresh_ports)
        btn_refresh.grid(row=0, column=2, padx=2)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, sticky=tk.W, padx=2)
        self.baud_combo = ttk.Combobox(conn_frame, width=8, values=["9600", "115200"], state="readonly")
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=4, padx=4)

        self.btn_connect = ttk.Button(conn_frame, text="Підключити", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=5, padx=6)

        # 2. Блок швидких дій (Start/Stop, Single, Info)
        actions_frame = ttk.LabelFrame(left_panel, text=" Швидкі команди ")
        actions_frame.pack(fill=tk.X, pady=(0, 8))

        self.btn_start = ttk.Button(actions_frame, text="▶ Start/Stop ", command=lambda: self.send_cmd("S"))
        self.btn_start.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_single = ttk.Button(actions_frame, text="🎯 Single ", command=lambda: self.send_cmd("R"))
        self.btn_single.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_info = ttk.Button(actions_frame, text="ℹ Status ", command=lambda: self.send_cmd("H"))
        self.btn_info.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # 3. БЛОК ПРЯМОГО ВИБОРУ ПАРАМЕТРІВ BMP280
        params_frame = ttk.LabelFrame(left_panel, text=" Прямий вибір параметрів ")
        params_frame.pack(fill=tk.X, pady=(0, 8))

        # 3.1. Інтервал опитування
        ttk.Label(params_frame, text="Інтервал:", style="Header.TLabel").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.interval_var = tk.StringVar(value="1")
        interval_opts = [("0.5 сек", "0", "I0"),("1 сек", "1", "I1"), ("5 сек", "2", "I2"), ("10 сек", "3", "I3")]

        i_frame = ttk.Frame(params_frame)
        i_frame.grid(row=0, column=1, columnspan=5, sticky=tk.W, pady=2)
        for text, val, cmd in interval_opts:
            rb = tk.Radiobutton(
                i_frame, text=text, value=val, variable=self.interval_var,
                indicatoron=0, width=8, selectcolor="#b3d9ff", bg="#e1e1e1",
                font=("Segoe UI", 9, "bold"), command=lambda c=cmd: self.send_cmd(c)
            )
            rb.pack(side=tk.LEFT, padx=2)

        # 3.2. IIR Фільтр
        ttk.Label(params_frame, text="IIR Фільтр:", style="Header.TLabel").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.filter_var = tk.StringVar(value="2")  # x4 за замовчуванням
        filter_opts = [("OFF", "0", "F0"), ("x2", "1", "F1"), ("x4", "2", "F2"), ("x8", "3", "F3"), ("x16", "4", "F4")]

        f_frame = ttk.Frame(params_frame)
        f_frame.grid(row=1, column=1, columnspan=5, sticky=tk.W, pady=2)
        for text, val, cmd in filter_opts:
            rb = tk.Radiobutton(
                f_frame, text=text, value=val, variable=self.filter_var,
                indicatoron=0, width=5, selectcolor="#b3d9ff", bg="#e1e1e1",
                font=("Segoe UI", 9, "bold"), command=lambda c=cmd: self.send_cmd(c)
            )
            rb.pack(side=tk.LEFT, padx=2)

        # 3.3. Оверсемплінг (Oversampling)
        ttk.Label(params_frame, text="Оверсемплінг:", style="Header.TLabel").grid(row=2, column=0, sticky=tk.W, pady=3)
        self.osrs_var = tk.StringVar(value="2")  # x2 за замовчуванням
        osrs_opts = [("x1", "0", "O0"), ("x2", "1", "O1"), ("x4", "2", "O2"), ("x8", "3", "O3"), ("x16", "4", "O4")]

        o_frame = ttk.Frame(params_frame)
        o_frame.grid(row=2, column=1, columnspan=5, sticky=tk.W, pady=2)
        for text, val, cmd in osrs_opts:
            rb = tk.Radiobutton(
                o_frame, text=text, value=val, variable=self.osrs_var,
                indicatoron=0, width=5, selectcolor="#b3d9ff", bg="#e1e1e1",
                font=("Segoe UI", 9, "bold"), command=lambda c=cmd: self.send_cmd(c)
            )
            rb.pack(side=tk.LEFT, padx=2)

        # 4. Консоль логування терміналу
        log_frame = ttk.LabelFrame(left_panel, text=" Термінал UART (TX/RX) ")
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_area = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, font=("Consolas", 9), bg="#1e1e1e",
                                                  fg="#00ff66")
        self.log_area.pack(fill=tk.BOTH, expand=True)

        # ==================== ПРАВА ПАНЕЛЬ ====================

        # Індикатори поточних значень
        ind_frame = ttk.Frame(right_panel)
        ind_frame.pack(fill=tk.X, pady=(0, 10))

        # Картка Температури
        t_card = tk.Frame(ind_frame, bg="#fff3e0", bd=2, relief="groove")
        t_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        tk.Label(t_card, text="ТЕМПЕРАТУРА", bg="#fff3e0", fg="#e65100", font=("Segoe UI", 10, "bold")).pack(
            anchor=tk.W, padx=8, pady=(4, 0))
        self.lbl_temp = tk.Label(t_card, text="--.- °C", bg="#fff3e0", fg="#d84315", font=("Segoe UI", 26, "bold"))
        self.lbl_temp.pack(padx=8, pady=(0, 4))

        # Картка Тиску
        p_card = tk.Frame(ind_frame, bg="#e1f5fe", bd=2, relief="groove")
        p_card.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        tk.Label(p_card, text="ТИСК", bg="#e1f5fe", fg="#0277bd", font=("Segoe UI", 10, "bold")).pack(anchor=tk.W,
                                                                                                      padx=8,
                                                                                                      pady=(4, 0))
        self.lbl_press = tk.Label(p_card, text="----.- hPa", bg="#e1f5fe", fg="#01579b", font=("Segoe UI", 26, "bold"))
        self.lbl_press.pack(padx=8, pady=(0, 4))

        # Динамічні графіки Matplotlib
        chart_frame = ttk.LabelFrame(right_panel, text=" Графіки в реальному часі ")
        chart_frame.pack(fill=tk.BOTH, expand=True)

        self.fig, (self.ax_temp, self.ax_press) = plt.subplots(2, 1, figsize=(6, 5), sharex=True)
        self.fig.patch.set_facecolor("#f5f5f5")

        self.line_temp, = self.ax_temp.plot([], [], 'o-', color="#d84315", linewidth=2, markersize=4, label="Temp (°C)")
        self.ax_temp.set_ylabel("°C", fontsize=9, fontweight='bold', color="#d84315")
        self.ax_temp.grid(True, linestyle="--", alpha=0.6)
        self.ax_temp.legend(loc="upper left")

        self.line_press, = self.ax_press.plot([], [], 's-', color="#01579b", linewidth=2, markersize=4,
                                              label="Press (hPa)")
        self.ax_press.set_ylabel("hPa", fontsize=9, fontweight='bold', color="#01579b")
        self.ax_press.set_xlabel("Час (секунди)", fontsize=9)
        self.ax_press.grid(True, linestyle="--", alpha=0.6)
        self.ax_press.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.refresh_ports()

    # ==================== ЛОГІКА ТА РОБОТА З UART ====================

    def log(self, msg, prefix="[SYS] "):
        timestamp = time.strftime("%H:%M:%S")
        self.log_area.insert(tk.END, f"{timestamp} {prefix}{msg}\n")
        self.log_area.see(tk.END)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
            self.log(f"Знайдено порти: {', '.join(ports)}")
        else:
            self.port_combo.set('')
            self.log("COM-порти не знайдені!", prefix="[WARN] ")

    def toggle_connection(self):
        if self.ser is None or not self.ser.is_open:
            port = self.port_combo.get()
            baud = self.baud_combo.get()

            if not port:
                messagebox.showerror("Помилка", "Оберіть COM-порт для підключення!")
                return

            try:
                self.ser = serial.Serial(port, int(baud), timeout=1.0)
                self.is_running = True
                self.start_time = time.time()

                self.read_thread = threading.Thread(target=self._reader_loop, daemon=True)
                self.read_thread.start()

                self.btn_connect.config(text="Відключити", style="Accent.TButton")
                self.log(f"Підключено до {port} на швидкості {baud} baud.")
            except Exception as e:
                messagebox.showerror("Помилка підключення", str(e))
                self.log(f"Не вдалося відкрити порт: {e}", prefix="[ERROR] ")
        else:
            self.is_running = False
            if self.ser:
                self.ser.close()
            self.btn_connect.config(text="Підключити")
            self.log("Відключено від порту.")

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('ascii'))
                self.log(f"Надіслано команду: '{cmd_str}'", prefix="[TX] ")
            except Exception as e:
                self.log(f"Помилка відправки: {e}", prefix="[ERROR] ")
        else:
            self.log("Неможливо надіслати команду: порт закритий!", prefix="[WARN] ")

    def _reader_loop(self):
        while self.is_running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.root.after(0, self.log, line, "[RX] ")
                    self._parse_line(line)
            except Exception as e:
                if self.is_running:
                    self.root.after(0, self.log, f"Помилка зчитання: {e}", "[ERROR] ")
                break

    def _parse_line(self, line):
        # Парсинг стандартної відповіді "T=24.5C P=1013.2hPa"
        match = re.search(r"T=([\d\.]+)\s*C\s+P=([\d\.]+)\s*hPa", line)
        if match:
            temp = float(match.group(1))
            press = float(match.group(2))
            elapsed = time.time() - self.start_time

            self.root.after(0, self._update_gui_data, temp, press, elapsed)

    def _update_gui_data(self, temp, press, elapsed):
        # Оновлення цифрових індикаторів
        self.lbl_temp.config(text=f"{temp:.1f} °C")
        self.lbl_press.config(text=f"{press:.1f} hPa")

        # Додавання даних у буфер для графіків
        self.time_data.append(elapsed)
        self.temp_data.append(temp)
        self.press_data.append(press)

        # Оновлення ліній на графіках
        self.line_temp.set_data(list(self.time_data), list(self.temp_data))
        self.line_press.set_data(list(self.time_data), list(self.press_data))

        # Автомасштабування осей
        self.ax_temp.relim()
        self.ax_temp.autoscale_view()
        self.ax_press.relim()
        self.ax_press.autoscale_view()

        self.canvas.draw_idle()

    def on_closing(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = BMP280ControlApp(root)
    root.mainloop()