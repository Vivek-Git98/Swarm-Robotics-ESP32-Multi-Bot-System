"""
==============================================================
 SWARM ROBOTICS — PC CONTROL GUI
 Python 3.8+  |  Requires: pyserial, tkinter (built-in)

 Install dependencies:
     pip install pyserial

 Usage:
     python swarm_control.py

 Serial Protocol (sent to Master Bot over USB):
     F<speed>  — Forward      B<speed>  — Backward
     L<speed>  — Turn Left    R<speed>  — Turn Right
     S         — Stop         A         — Auto-Follow mode
     M         — Manual mode  ?         — Request telemetry
==============================================================
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import json
import time
import math

# ──────────────────────────────────────────────────────────────
#  CONFIG
# ──────────────────────────────────────────────────────────────
BAUD_RATE        = 115200
TELEMETRY_POLL   = 200       # ms between telemetry requests
MAP_SCALE        = 3.0       # pixels per cm
MAP_W            = 600       # canvas width  (px)
MAP_H            = 500       # canvas height (px)
TRAIL_MAX        = 300       # max trail dots per bot
BOT_COLORS       = {1: "#FF4444", 2: "#44AAFF", 3: "#44FF88"}
BOT_LABELS       = {1: "MASTER", 2: "SLAVE 1", 3: "SLAVE 2"}

# ──────────────────────────────────────────────────────────────
#  SERIAL MANAGER
# ──────────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self):
        self.port   = None
        self.conn   = None
        self.lock   = threading.Lock()
        self.running= False
        self.on_msg = None   # callback(dict)

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        try:
            self.conn = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self.port = port
            self.running = True
            threading.Thread(target=self._reader, daemon=True).start()
            return True
        except Exception as e:
            return str(e)

    def disconnect(self):
        self.running = False
        if self.conn and self.conn.is_open:
            self.conn.close()

    def send(self, cmd: str):
        if self.conn and self.conn.is_open:
            with self.lock:
                try:
                    self.conn.write((cmd + "\n").encode())
                except Exception:
                    pass

    def _reader(self):
        buf = ""
        while self.running:
            try:
                if self.conn.in_waiting:
                    buf += self.conn.read(self.conn.in_waiting).decode(errors="replace")
                    lines = buf.split("\n")
                    buf = lines[-1]
                    for line in lines[:-1]:
                        line = line.strip()
                        if line.startswith("{"):
                            try:
                                data = json.loads(line)
                                if self.on_msg:
                                    self.on_msg(data)
                            except json.JSONDecodeError:
                                pass
            except Exception:
                pass
            time.sleep(0.02)


# ──────────────────────────────────────────────────────────────
#  BOT STATE MODEL
# ──────────────────────────────────────────────────────────────
class BotState:
    def __init__(self, bot_id):
        self.id          = bot_id
        self.x           = 0.0
        self.y           = 0.0
        self.heading     = 0.0
        self.state       = 0
        self.velocity    = 0.0
        self.wp          = 0
        self.alive       = False
        self.trail       = []   # list of (x,y) canvas coords

    STATE_NAMES = {0: "MANUAL", 1: "AUTO-FOLLOW", 2: "STOPPED"}

    def update(self, d: dict):
        self.x        = d.get("x", self.x)
        self.y        = d.get("y", self.y)
        self.heading  = d.get("hdg", self.heading)
        self.state    = d.get("state", self.state)
        self.velocity = d.get("vel", self.velocity)
        self.wp       = d.get("wp", self.wp)
        self.alive    = True


# ──────────────────────────────────────────────────────────────
#  MAIN GUI
# ──────────────────────────────────────────────────────────────
class SwarmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("🤖 Swarm Robotics Control Panel")
        self.configure(bg="#0D1117")
        self.resizable(False, False)

        self.serial = SerialManager()
        self.serial.on_msg = self._on_telemetry

        self.bots  = {i: BotState(i) for i in range(1, 4)}
        self.speed = tk.IntVar(value=160)
        self.mode  = tk.StringVar(value="MANUAL")

        # Track held keys for smooth drive
        self._keys = set()

        self._build_ui()
        self._bind_keys()
        self._start_loops()

    # ── UI CONSTRUCTION ──────────────────────────────────────
    def _build_ui(self):
        # ── Top: Connection bar ──────────────────────────────
        conn_frame = tk.Frame(self, bg="#161B22", pady=6, padx=10)
        conn_frame.pack(fill=tk.X, side=tk.TOP)

        tk.Label(conn_frame, text="PORT:", bg="#161B22", fg="#8B949E",
                 font=("Courier", 10)).pack(side=tk.LEFT)

        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(conn_frame, textvariable=self.port_var,
                                      width=14, state="readonly")
        self.port_cb.pack(side=tk.LEFT, padx=6)
        self._refresh_ports()

        tk.Button(conn_frame, text="⟳", command=self._refresh_ports,
                  bg="#21262D", fg="#C9D1D9", relief=tk.FLAT,
                  font=("Courier", 11), padx=4).pack(side=tk.LEFT)

        self.conn_btn = tk.Button(conn_frame, text="  CONNECT  ",
                                   command=self._toggle_connect,
                                   bg="#238636", fg="white",
                                   relief=tk.FLAT, font=("Courier", 10, "bold"),
                                   padx=8)
        self.conn_btn.pack(side=tk.LEFT, padx=8)

        self.conn_status = tk.Label(conn_frame, text="● DISCONNECTED",
                                     bg="#161B22", fg="#DA3633",
                                     font=("Courier", 10, "bold"))
        self.conn_status.pack(side=tk.LEFT, padx=6)

        # ── Main layout: left=map, right=controls ────────────
        main_frame = tk.Frame(self, bg="#0D1117")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)

        # MAP canvas
        map_outer = tk.Frame(main_frame, bg="#161B22", bd=0)
        map_outer.pack(side=tk.LEFT, padx=(0,10))

        tk.Label(map_outer, text="POSITION MAP  (cm)",
                 bg="#161B22", fg="#8B949E",
                 font=("Courier", 9)).pack(anchor=tk.W, padx=6, pady=(4,0))

        self.canvas = tk.Canvas(map_outer, width=MAP_W, height=MAP_H,
                                 bg="#0D1117", highlightthickness=1,
                                 highlightbackground="#30363D")
        self.canvas.pack(padx=6, pady=6)
        self._draw_grid()

        # RIGHT panel
        right = tk.Frame(main_frame, bg="#0D1117")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Mode selector
        self._build_mode_panel(right)

        # Bot status cards
        self._build_bot_cards(right)

        # Joystick
        self._build_joystick(right)

        # Speed slider
        self._build_speed_ctrl(right)

        # Log area
        self._build_log(right)

    def _build_mode_panel(self, parent):
        f = tk.LabelFrame(parent, text=" DRIVE MODE ", bg="#0D1117", fg="#8B949E",
                          font=("Courier", 9), bd=1, relief=tk.GROOVE,
                          highlightbackground="#30363D")
        f.pack(fill=tk.X, pady=(0, 8))
        inner = tk.Frame(f, bg="#0D1117"); inner.pack(pady=6, padx=8)

        for mode, cmd, color in [("MANUAL", "M", "#F0883E"),
                                   ("AUTO-FOLLOW", "A", "#3FB950"),
                                   ("STOP ALL", "S", "#DA3633")]:
            tk.Button(inner, text=mode, width=12,
                      command=lambda c=cmd, m=mode: self._set_mode(c, m),
                      bg=color, fg="white", relief=tk.FLAT,
                      font=("Courier", 10, "bold"), padx=4
                      ).pack(side=tk.LEFT, padx=4)

    def _build_bot_cards(self, parent):
        f = tk.LabelFrame(parent, text=" BOT STATUS ", bg="#0D1117", fg="#8B949E",
                          font=("Courier", 9), bd=1, relief=tk.GROOVE)
        f.pack(fill=tk.X, pady=(0, 8))

        self.bot_labels = {}
        for i in range(1, 4):
            row = tk.Frame(f, bg="#161B22", pady=4, padx=8)
            row.pack(fill=tk.X, padx=6, pady=3)

            dot = tk.Label(row, text="●", fg=BOT_COLORS[i],
                           bg="#161B22", font=("Courier", 14))
            dot.pack(side=tk.LEFT)

            tk.Label(row, text=f" {BOT_LABELS[i]} ", bg="#161B22",
                     fg="#C9D1D9", font=("Courier", 10, "bold")).pack(side=tk.LEFT)

            info = tk.Label(row, text="--", bg="#161B22", fg="#8B949E",
                            font=("Courier", 9), anchor=tk.W)
            info.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.bot_labels[i] = info

    def _build_joystick(self, parent):
        f = tk.LabelFrame(parent, text=" KEYBOARD / JOYSTICK  [WASD]",
                          bg="#0D1117", fg="#8B949E",
                          font=("Courier", 9), bd=1, relief=tk.GROOVE)
        f.pack(fill=tk.X, pady=(0, 8))

        grid = tk.Frame(f, bg="#0D1117"); grid.pack(pady=8)

        btn_style = dict(width=4, height=2, relief=tk.FLAT,
                         font=("Courier", 12, "bold"),
                         bg="#21262D", fg="#C9D1D9",
                         activebackground="#30363D")

        tk.Button(grid, text="▲", command=lambda: self._drive("F"), **btn_style
                  ).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(grid, text="◄", command=lambda: self._drive("L"), **btn_style
                  ).grid(row=1, column=0, padx=3, pady=3)
        tk.Button(grid, text="■", command=lambda: self._drive("S"),
                  bg="#DA3633", fg="white",
                  width=4, height=2, relief=tk.FLAT,
                  font=("Courier", 12, "bold")
                  ).grid(row=1, column=1, padx=3, pady=3)
        tk.Button(grid, text="►", command=lambda: self._drive("R"), **btn_style
                  ).grid(row=1, column=2, padx=3, pady=3)
        tk.Button(grid, text="▼", command=lambda: self._drive("B"), **btn_style
                  ).grid(row=2, column=1, padx=3, pady=3)

        tk.Label(f, text="WASD keys also work when window is focused",
                 bg="#0D1117", fg="#484F58", font=("Courier", 8)
                 ).pack(pady=(0,6))

    def _build_speed_ctrl(self, parent):
        f = tk.LabelFrame(parent, text=" SPEED ", bg="#0D1117", fg="#8B949E",
                          font=("Courier", 9), bd=1, relief=tk.GROOVE)
        f.pack(fill=tk.X, pady=(0, 8))

        inner = tk.Frame(f, bg="#0D1117"); inner.pack(fill=tk.X, padx=10, pady=8)
        self.speed_label = tk.Label(inner, textvariable=self.speed,
                                    bg="#0D1117", fg="#F0883E",
                                    font=("Courier", 18, "bold"), width=4)
        self.speed_label.pack(side=tk.LEFT)

        tk.Scale(inner, from_=60, to=255,
                 orient=tk.HORIZONTAL, variable=self.speed,
                 bg="#0D1117", fg="#C9D1D9", troughcolor="#21262D",
                 highlightthickness=0, sliderlength=20,
                 showvalue=0
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)

    def _build_log(self, parent):
        f = tk.LabelFrame(parent, text=" LOG ", bg="#0D1117", fg="#8B949E",
                          font=("Courier", 9), bd=1, relief=tk.GROOVE)
        f.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(f, height=6, bg="#0D1117", fg="#8B949E",
                           font=("Courier", 8), relief=tk.FLAT,
                           state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

    # ── GRID / MAP ────────────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("grid")
        for x in range(0, MAP_W, int(50 * MAP_SCALE / 10)):
            self.canvas.create_line(x, 0, x, MAP_H, fill="#1C2128", tags="grid")
        for y in range(0, MAP_H, int(50 * MAP_SCALE / 10)):
            self.canvas.create_line(0, y, MAP_W, y, fill="#1C2128", tags="grid")
        # Origin marker
        ox, oy = MAP_W // 2, MAP_H // 2
        self.canvas.create_oval(ox-4, oy-4, ox+4, oy+4, fill="#30363D", outline="", tags="grid")

    def _world_to_canvas(self, x, y):
        cx = MAP_W // 2 + x * MAP_SCALE
        cy = MAP_H // 2 - y * MAP_SCALE   # Y flipped
        return cx, cy

    def _update_map(self):
        self.canvas.delete("bot", "trail")
        for i, bot in self.bots.items():
            if not bot.alive:
                continue
            cx, cy = self._world_to_canvas(bot.x, bot.y)
            col = BOT_COLORS[i]

            # Trail
            bot.trail.append((cx, cy))
            if len(bot.trail) > TRAIL_MAX:
                bot.trail.pop(0)
            for j in range(1, len(bot.trail)):
                alpha = j / len(bot.trail)
                self.canvas.create_line(
                    bot.trail[j-1][0], bot.trail[j-1][1],
                    bot.trail[j][0],   bot.trail[j][1],
                    fill=col, width=1, tags="trail",
                    stipple="gray25" if alpha < 0.5 else ""
                )

            # Heading arrow
            rad  = math.radians(bot.heading)
            ax   = cx + 18 * math.cos(rad)
            ay   = cy - 18 * math.sin(rad)
            self.canvas.create_line(cx, cy, ax, ay, fill=col, width=2,
                                     arrow=tk.LAST, arrowshape=(8,10,4), tags="bot")

            # Bot circle
            r = 10
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                     fill=col, outline="#0D1117", width=2, tags="bot")
            self.canvas.create_text(cx, cy, text=str(i),
                                     fill="white", font=("Courier", 8, "bold"), tags="bot")

    # ── TELEMETRY ─────────────────────────────────────────────
    def _on_telemetry(self, d: dict):
        bid = d.get("id")
        if bid in self.bots:
            self.bots[bid].update(d)

    def _poll_telemetry(self):
        self.serial.send("?")
        self.after(TELEMETRY_POLL, self._poll_telemetry)

    def _update_bot_cards(self):
        for i, bot in self.bots.items():
            if bot.alive:
                txt = (f"x={bot.x:6.1f}cm  y={bot.y:6.1f}cm  "
                       f"hdg={bot.heading:5.1f}°  "
                       f"vel={bot.velocity:4.1f}cm/s  "
                       f"[{BotState.STATE_NAMES.get(bot.state, '?')}]")
            else:
                txt = "  ── not connected ──"
            self.bot_labels[i].config(text=txt,
                                       fg="#3FB950" if bot.alive else "#484F58")

    # ── DRIVE COMMANDS ────────────────────────────────────────
    def _drive(self, direction):
        spd = self.speed.get()
        if direction == "S":
            self.serial.send("S")
        else:
            self.serial.send(f"{direction}{spd}")

    def _set_mode(self, cmd, name):
        self.serial.send(cmd)
        self.mode.set(name)
        self._log(f"Mode → {name}")

    # ── KEY BINDINGS ──────────────────────────────────────────
    def _bind_keys(self):
        self.bind("<KeyPress-w>",   lambda e: self._key_press("F"))
        self.bind("<KeyPress-s>",   lambda e: self._key_press("B"))
        self.bind("<KeyPress-a>",   lambda e: self._key_press("L"))
        self.bind("<KeyPress-d>",   lambda e: self._key_press("R"))
        self.bind("<KeyRelease-w>", lambda e: self._key_release("F"))
        self.bind("<KeyRelease-s>", lambda e: self._key_release("B"))
        self.bind("<KeyRelease-a>", lambda e: self._key_release("L"))
        self.bind("<KeyRelease-d>", lambda e: self._key_release("R"))
        self.bind("<space>",        lambda e: self._drive("S"))
        self.bind("<KeyPress-Up>",    lambda e: self._key_press("F"))
        self.bind("<KeyPress-Down>",  lambda e: self._key_press("B"))
        self.bind("<KeyPress-Left>",  lambda e: self._key_press("L"))
        self.bind("<KeyPress-Right>", lambda e: self._key_press("R"))

    def _key_press(self, d):
        if d not in self._keys:
            self._keys.add(d)
            self._drive(d)

    def _key_release(self, d):
        self._keys.discard(d)
        if not self._keys:
            self.serial.send("S")

    # ── CONNECTION ────────────────────────────────────────────
    def _refresh_ports(self):
        ports = SerialManager.list_ports()
        self.port_cb["values"] = ports
        if ports:
            self.port_cb.current(0)

    def _toggle_connect(self):
        if self.serial.conn and self.serial.conn.is_open:
            self.serial.disconnect()
            self.conn_btn.config(text="  CONNECT  ", bg="#238636")
            self.conn_status.config(text="● DISCONNECTED", fg="#DA3633")
            self._log("Disconnected")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showwarning("No Port", "Select a serial port first.")
                return
            result = self.serial.connect(port)
            if result is True:
                self.conn_btn.config(text="  DISCONNECT  ", bg="#DA3633")
                self.conn_status.config(text=f"● CONNECTED  {port}", fg="#3FB950")
                self._log(f"Connected → {port} @ {BAUD_RATE}")
            else:
                messagebox.showerror("Connection Error", str(result))

    # ── LOG ───────────────────────────────────────────────────
    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, f"[{ts}] {msg}\n")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    # ── ANIMATION LOOPS ───────────────────────────────────────
    def _start_loops(self):
        self.after(TELEMETRY_POLL, self._poll_telemetry)
        self._render_loop()

    def _render_loop(self):
        self._update_map()
        self._update_bot_cards()
        self.after(50, self._render_loop)   # ~20 fps


# ──────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = SwarmGUI()
    app.mainloop()
