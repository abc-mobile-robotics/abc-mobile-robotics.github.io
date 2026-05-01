#!/usr/bin/env python3
"""
Wolf FSM parameter tuner — slider + text entry edition, with Kalman filter tuning.

Usage:
    python3 wolf_tuner.py

Requires wolf_fsm to have add_on_set_parameters_callback wired up.

Notes:
  * chase_kp/ki/kd operate on NORMALIZED error (-1..+1 across the frame).
  * kf_* parameters live in pixel units.
  * The gauge shows both the raw YOLO measurement (dim dot) and the filter's
    current estimate + lookahead (bright bar) so you can watch the KF work.
"""

import threading
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String

TARGET_NODE = '/wolf_fsm'

# ── Parameter definitions ──────────────────────────────────────────────────────
# (name, lo, hi, step, default, is_int, section)
PARAMS: List[Tuple[str, float, float, float, float, bool, str]] = [
    # Rotation / PID
    ('chase_kp',               0.0,    8.0,     0.05,    2.0,    False, 'rotation'),
    ('chase_ki',               0.0,    2.0,     0.01,    0.0,    False, 'rotation'),
    ('chase_kd',               0.0,    1.0,     0.01,    0.15,   False, 'rotation'),
    ('chase_deadband',         0.0,    0.2,     0.005,   0.02,   False, 'rotation'),
    ('max_angular_speed',      0.0,    4.0,     0.05,    1.5,    False, 'rotation'),
    ('center_x_px',            50.0,   400.0,   1.0,     125.0,  False, 'rotation'),
    # Kalman filter
    ('kf_pixels_per_rad',      50.0,   600.0,   1.0,     240.0,  False, 'kalman'),
    ('kf_process_px_var',      0.0,    500.0,   1.0,     50.0,   False, 'kalman'),
    ('kf_process_vx_var',      0.0,    20000.0, 50.0,    2500.0, False, 'kalman'),
    ('kf_meas_var',            1.0,    500.0,   1.0,     25.0,   False, 'kalman'),
    ('kf_feedforward_sec',     0.0,    0.5,     0.01,    0.15,   False, 'kalman'),
    ('kf_sign',               -1.0,    1.0,     2.0,     1.0,    False, 'kalman'),
    ('control_hz',             10.0,   100.0,   5.0,     50.0,   False, 'kalman'),
    # Speed
    ('chase_linear_speed',     0.0,    2.5,     0.05,    0.0,    False, 'speed'),
    ('patrol_linear_speed',    0.0,    0.5,     0.01,    0.10,   False, 'speed'),
    ('patrol_turn_speed',      0.0,    1.5,     0.05,    0.65,   False, 'speed'),
    ('return_turn_speed',      0.0,    1.5,     0.05,    0.75,   False, 'speed'),
    # Misc
    ('vision_timeout_sec',     0.1,    2.0,     0.05,    0.7,    False, 'misc'),
    ('chase_lost_frames',      1,      20,      1,       5,      True,  'misc'),
]

CHECKBOXES: List[Tuple[str, bool, str]] = [
    ('chase_speed_scale_on_turn', True, 'speed'),
]

SECTION_COLORS = {
    'rotation': '#1a3a5c',
    'kalman':   '#4a2a5a',
    'speed':    '#1a3a1a',
    'misc':     '#3a2a4a',
}
SECTION_LABELS = {
    'rotation': '  Rotation / Tracking PID',
    'kalman':   '  Kalman Filter',
    'speed':    '  Speed',
    'misc':     '  Misc',
}
SECTION_ORDER = ('rotation', 'kalman', 'speed', 'misc')


# ── ROS 2 client node ──────────────────────────────────────────────────────────

class ParamClient(Node):
    def __init__(self, target: str) -> None:
        super().__init__('wolf_tuner_client')
        self.target = target.lstrip('/')
        self.set_cli = self.create_client(SetParameters, f'/{self.target}/set_parameters')
        self.get_cli = self.create_client(GetParameters, f'/{self.target}/get_parameters')

        self._vision: dict = {}
        self._vision_lock = threading.Lock()
        self.create_subscription(String, '/wolf/vision', self._vision_cb, 10)

    def _vision_cb(self, msg: String) -> None:
        # Try JSON first (that's what the real vision node publishes),
        # fall back to "key=value key=value" for older/simpler publishers.
        raw = (msg.data or '').strip()
        d: dict = {}
        if raw.startswith('{'):
            try:
                import json
                parsed = json.loads(raw)
                if isinstance(parsed, dict):
                    d = parsed
            except (ValueError, TypeError):
                d = {}
        if not d:
            for token in raw.split():
                if '=' in token:
                    k, _, v = token.partition('=')
                    d[k] = v
        with self._vision_lock:
            self._vision = d

    def get_vision(self) -> dict:
        with self._vision_lock:
            return dict(self._vision)

    def wait_ready(self, timeout_sec: float = 5.0) -> bool:
        return (self.set_cli.wait_for_service(timeout_sec=timeout_sec)
                and self.get_cli.wait_for_service(timeout_sec=timeout_sec))

    def set_double(self, name: str, value: float) -> bool:
        p = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        return self._call_set([p])

    def set_int(self, name: str, value: int) -> bool:
        p = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=int(value)))
        return self._call_set([p])

    def set_bool(self, name: str, value: bool) -> bool:
        p = Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)))
        return self._call_set([p])

    def _call_set(self, params: List[Parameter]) -> bool:
        req = SetParameters.Request()
        req.parameters = params
        future = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return False
        return all(r.successful for r in future.result().results)

    def get_current(self, names: List[str]) -> Dict[str, float]:
        req = GetParameters.Request()
        req.names = names
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        out: Dict[str, float] = {}
        if future.result() is None:
            return out
        for name, pv in zip(names, future.result().values):
            t = pv.type
            if t == ParameterType.PARAMETER_DOUBLE:
                out[name] = pv.double_value
            elif t == ParameterType.PARAMETER_INTEGER:
                out[name] = pv.integer_value
            elif t == ParameterType.PARAMETER_BOOL:
                out[name] = pv.bool_value
        return out

    def spin_background(self) -> None:
        rclpy.spin(self)


# ── Tuner-side Kalman filter (mirror of the FSM's KF for display only) ────────
#
# The FSM does the real filtering. This copy lets the gauge show a smoothed
# track without waiting for the node to publish its internal state. If you want
# the gauge to display the FSM's actual internal estimate, have the FSM publish
# a topic like /wolf/kf_state and subscribe to it instead.

class _DisplayKF:
    def __init__(self) -> None:
        self.px = 0.0
        self.vx = 0.0
        self.P = [1e6, 0.0, 0.0, 1e6]
        self.initialized = False
        self.pixels_per_rad = 240.0
        self.process_px_var = 50.0
        self.process_vx_var = 2500.0
        self.meas_var = 25.0
        self.sign = 1.0

    def reset(self) -> None:
        self.__init__()

    def predict(self, dt: float, omega_cmd: float) -> None:
        if dt <= 0.0 or not self.initialized:
            return
        self.px = self.px + self.vx * dt - omega_cmd * self.sign * dt * self.pixels_per_rad
        p00, p01, p10, p11 = self.P
        a00 = p00 + dt * p10; a01 = p01 + dt * p11
        a10 = p10;            a11 = p11
        n00 = a00 + a01 * dt; n01 = a01
        n10 = a10 + a11 * dt; n11 = a11
        n00 += self.process_px_var * dt
        n11 += self.process_vx_var * dt
        self.P = [n00, n01, n10, n11]

    def update(self, z: float, conf: float) -> None:
        conf = max(0.1, min(1.0, conf))
        R = self.meas_var / (conf * conf)
        if not self.initialized:
            self.px = z
            self.vx = 0.0
            self.P = [R, 0.0, 0.0, 1e4]
            self.initialized = True
            return
        p00, p01, p10, p11 = self.P
        S = p00 + R
        if S <= 0.0:
            return
        k0 = p00 / S
        k1 = p10 / S
        y = z - self.px
        self.px += k0 * y
        self.vx += k1 * y
        n00 = (1.0 - k0) * p00
        n01 = (1.0 - k0) * p01
        n10 = -k1 * p00 + p10
        n11 = -k1 * p01 + p11
        self.P = [n00, n01, n10, n11]


# ── One parameter row ──────────────────────────────────────────────────────────

class ParamRow:
    def __init__(self, parent: tk.Widget, client: ParamClient,
                 name: str, lo: float, hi: float, step: float,
                 initial: float, is_int: bool) -> None:
        self.client  = client
        self.name    = name
        self.lo      = lo
        self.hi      = hi
        self.step    = step
        self.is_int  = is_int
        self._guard  = False

        self.var = tk.IntVar(value=int(initial)) if is_int else tk.DoubleVar(value=initial)

        frame = ttk.Frame(parent)
        frame.pack(fill='x', padx=10, pady=2)

        ttk.Label(frame, text=name, width=25, anchor='w',
                  font=('Courier', 9)).pack(side='left')

        self.scale = ttk.Scale(frame, from_=lo, to=hi, orient='horizontal',
                               variable=self.var, command=self._slider_moved)
        self.scale.pack(side='left', fill='x', expand=True, padx=6)

        self.entry_sv = tk.StringVar(value=self._fmt(initial))
        self.entry = ttk.Entry(frame, textvariable=self.entry_sv, width=10,
                               justify='right', font=('Courier', 9))
        self.entry.pack(side='left', padx=(0, 4))
        self.entry.bind('<Return>',   self._entry_committed)
        self.entry.bind('<FocusOut>', self._entry_committed)
        self.entry.bind('<Up>',       lambda _: self._nudge(+self.step))
        self.entry.bind('<Down>',     lambda _: self._nudge(-self.step))

        self.dot = tk.Label(frame, text='●', fg='#444444',
                            bg='#1e1e1e', font=('', 9), width=2)
        self.dot.pack(side='left')

    def _fmt(self, v: float) -> str:
        if self.is_int:
            return str(int(round(v)))
        if self.step >= 10.0:
            return f'{v:.0f}'
        if self.step >= 1.0:
            return f'{v:.1f}'
        if self.step >= 0.05:
            return f'{v:.3f}'
        return f'{v:.5f}'

    def _clamp(self, v: float) -> float:
        return max(self.lo, min(self.hi, v))

    def _push(self, value: float) -> None:
        ok = self.client.set_int(self.name, int(value)) if self.is_int \
             else self.client.set_double(self.name, float(value))
        self.dot.config(fg='#00cc44' if ok else '#cc2200')
        self.dot.after(700, lambda: self.dot.config(fg='#444444'))

    def _slider_moved(self, _=None) -> None:
        if self._guard:
            return
        self._guard = True
        v = self._clamp(self.var.get())
        self.entry_sv.set(self._fmt(v))
        self._push(v)
        self._guard = False

    def _entry_committed(self, _=None) -> None:
        if self._guard:
            return
        raw = self.entry_sv.get().strip()
        try:
            v = int(raw) if self.is_int else float(raw)
        except ValueError:
            self.entry.config(style='Err.TEntry')
            self.entry_sv.set(self._fmt(self._clamp(self.var.get())))
            return
        v = self._clamp(v)
        self._guard = True
        self.var.set(v)
        self.entry_sv.set(self._fmt(v))
        self._push(v)
        self._guard = False

    def _nudge(self, delta: float) -> None:
        raw = self.entry_sv.get().strip()
        try:
            v = int(raw) if self.is_int else float(raw)
        except ValueError:
            return
        self._guard = True
        v = self._clamp(v + delta)
        self.var.set(v)
        self.entry_sv.set(self._fmt(v))
        self._push(v)
        self._guard = False

    def set_value(self, v: float, push: bool = False) -> None:
        prev_guard = self._guard
        self._guard = True
        v = self._clamp(v)
        self.var.set(v)
        self.entry_sv.set(self._fmt(v))
        if push:
            self._push(v)
        self._guard = prev_guard

    def disable(self) -> None:
        self._guard = True
        self.scale.state(['disabled'])
        self.entry.config(state='disabled')

    def enable(self) -> None:
        self._guard = False
        self.scale.state(['!disabled'])
        self.entry.config(state='normal')


# ── Main GUI ───────────────────────────────────────────────────────────────────

class TunerGUI:
    def __init__(self, client: ParamClient, connected: bool) -> None:
        self.client = client
        self.rows: Dict[str, ParamRow] = {}
        self.bool_vars: Dict[str, tk.BooleanVar] = {}

        # Display-side KF (mirrors FSM math so gauge can show filtered track)
        self._dkf = _DisplayKF()
        self._last_dkf_time: float | None = None
        self._last_vision_stamp: str | None = None

        self.root = tk.Tk()
        self.root.title('Wolf FSM Tuner')
        self.root.geometry('740x960')
        self.root.configure(bg='#1e1e1e')

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.',            background='#1e1e1e', foreground='#e0e0e0')
        style.configure('TFrame',       background='#1e1e1e')
        style.configure('TLabel',       background='#1e1e1e', foreground='#e0e0e0')
        style.configure('TScale',       background='#1e1e1e', troughcolor='#3a3a3a',
                        sliderlength=16)
        style.configure('TEntry',       fieldbackground='#2e2e2e', foreground='#e0e0e0',
                        insertcolor='#e0e0e0')
        style.configure('Err.TEntry',   fieldbackground='#4a1010', foreground='#ff8080')
        style.configure('TCheckbutton', background='#1e1e1e', foreground='#e0e0e0')
        style.configure('TButton',      background='#333333', foreground='#e0e0e0', padding=4)
        style.map('TButton',            background=[('active', '#555555')])
        style.configure('TScrollbar',   background='#333333', troughcolor='#1e1e1e',
                        arrowcolor='#999')

        # Header
        hdr = ttk.Frame(self.root, padding=(12, 8, 12, 4))
        hdr.pack(fill='x')
        tk.Label(hdr, text='Wolf FSM Tuner', fg='#e0e0e0', bg='#1e1e1e',
                 font=('', 13, 'bold')).pack(side='left')
        dot_color = '#00cc44' if connected else '#cc8800'
        dot_text  = f'● {TARGET_NODE}' if connected else '● offline — defaults shown'
        tk.Label(hdr, text=dot_text, fg=dot_color, bg='#1e1e1e',
                 font=('', 9)).pack(side='right')

        # Rotation-only toggle
        lock_bar = ttk.Frame(self.root, padding=(12, 2, 12, 4))
        lock_bar.pack(fill='x')
        self._lock_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            lock_bar,
            text='  Rotation-only mode  —  locks chase_linear_speed to 0  (great for PID/KF tuning)',
            variable=self._lock_var,
            command=self._on_lock_toggle,
        ).pack(side='left')

        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(2, 0))

        # Scrollable parameter area
        outer = ttk.Frame(self.root)
        outer.pack(fill='both', expand=True)
        canvas = tk.Canvas(outer, bg='#1e1e1e', highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
        self._sf = ttk.Frame(canvas)
        self._sf.bind('<Configure>',
                      lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=self._sf, anchor='nw')
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side='left', fill='both', expand=True)
        vsb.pack(side='right', fill='y')
        canvas.bind_all('<MouseWheel>',
                        lambda e: canvas.yview_scroll(-1*(e.delta//120), 'units'))

        # Fetch live values
        all_names = [p[0] for p in PARAMS] + [c[0] for c in CHECKBOXES]
        current = client.get_current(all_names) if connected else {}

        # Build sections
        for section in SECTION_ORDER:
            self._section_header(section)
            for (name, lo, hi, step, default, is_int, sec) in PARAMS:
                if sec != section:
                    continue
                val = current.get(name, default)
                row = ParamRow(self._sf, client, name, lo, hi, step, val, is_int)
                self.rows[name] = row
            for (name, default, sec) in CHECKBOXES:
                if sec != section:
                    continue
                val = bool(current.get(name, default))
                self._checkbox_row(name, val)

        # Live gauge
        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(4, 0))
        self._build_gauge()

        # Footer
        footer = ttk.Frame(self.root, padding=(8, 6))
        footer.pack(fill='x', side='bottom')
        ttk.Button(footer, text='Reload from node',
                   command=self.reload).pack(side='left')
        ttk.Button(footer, text='Reset defaults',
                   command=self.reset_defaults).pack(side='left', padx=6)
        ttk.Button(footer, text='Reset KF',
                   command=self._reset_display_kf).pack(side='left', padx=6)
        ttk.Button(footer, text='Quit',
                   command=self.root.quit).pack(side='right')

        if self._lock_var.get():
            self._on_lock_toggle()

        self._refresh_gauge()

    def _section_header(self, section: str) -> None:
        color = SECTION_COLORS.get(section, '#333333')
        text  = SECTION_LABELS.get(section, section.upper())
        bar = tk.Frame(self._sf, bg=color, pady=5)
        bar.pack(fill='x', pady=(10, 2))
        tk.Label(bar, text=text, bg=color, fg='white',
                 font=('', 10, 'bold'), padx=12).pack(side='left')

    def _checkbox_row(self, name: str, val: bool) -> None:
        frame = ttk.Frame(self._sf)
        frame.pack(fill='x', padx=10, pady=2)
        bv = tk.BooleanVar(value=val)
        self.bool_vars[name] = bv

        def _cb(n=name, v=bv):
            self.client.set_bool(n, v.get())

        ttk.Checkbutton(frame, text=name, variable=bv, command=_cb).pack(side='left')

    def _on_lock_toggle(self) -> None:
        locked = self._lock_var.get()
        row = self.rows.get('chase_linear_speed')
        if not isinstance(row, ParamRow):
            return
        if locked:
            row.disable()
            row.set_value(0.0)
            self.client.set_double('chase_linear_speed', 0.0)
        else:
            row.enable()

    def _reset_display_kf(self) -> None:
        self._dkf.reset()
        self._last_vision_stamp = None
        self._last_dkf_time = None

    # ── live gauge ────────────────────────────────────────────────────────────

    def _build_gauge(self) -> None:
        g = ttk.Frame(self.root, padding=(12, 6, 12, 4))
        g.pack(fill='x')

        tk.Label(g,
                 text='Live tracking   /wolf/vision    raw = dim dot   KF+lookahead = bright bar',
                 fg='#888888', bg='#1e1e1e', font=('', 9, 'bold')).pack(anchor='w')

        self._gcanvas = tk.Canvas(g, height=42, bg='#242424',
                                  highlightthickness=1, highlightbackground='#444444')
        self._gcanvas.pack(fill='x', pady=(4, 4))

        row2 = tk.Frame(g, bg='#1e1e1e')
        row2.pack(fill='x')

        self._lbl_err  = tk.Label(row2, text='err (KF) :  ---',
                                  fg='#888888', bg='#1e1e1e',
                                  font=('Courier', 9), width=22, anchor='w')
        self._lbl_err.pack(side='left')
        self._lbl_vx   = tk.Label(row2, text='vx : --- px/s',
                                  fg='#888888', bg='#1e1e1e',
                                  font=('Courier', 9), width=18, anchor='w')
        self._lbl_vx.pack(side='left')
        self._lbl_vis  = tk.Label(row2, text='visible : ---',
                                  fg='#888888', bg='#1e1e1e',
                                  font=('Courier', 9), width=15, anchor='w')
        self._lbl_vis.pack(side='left')
        self._lbl_state = tk.Label(row2, text='',
                                   fg='#ffcc00', bg='#1e1e1e',
                                   font=('Courier', 9, 'bold'))
        self._lbl_state.pack(side='right')

    def _row_val(self, name: str, default: float) -> float:
        r = self.rows.get(name)
        try:
            return float(r.var.get()) if r else default
        except Exception:
            return default

    def _refresh_gauge(self) -> None:
        try:
            # Pull latest slider values into the display-side KF
            self._dkf.pixels_per_rad = self._row_val('kf_pixels_per_rad', 240.0)
            self._dkf.process_px_var = self._row_val('kf_process_px_var', 50.0)
            self._dkf.process_vx_var = self._row_val('kf_process_vx_var', 2500.0)
            self._dkf.meas_var       = self._row_val('kf_meas_var', 25.0)
            self._dkf.sign           = self._row_val('kf_sign', 1.0)

            v        = self.client.get_vision()
            rv       = v.get('rabbit_visible', False)
            # rabbit_visible may arrive as bool (from JSON) or str (from key=value)
            if isinstance(rv, bool):
                visible = rv
            else:
                visible = str(rv).strip().lower() == 'true'
            cx_raw   = v.get('rabbit_center_x', None)
            iw_raw   = v.get('image_width', None)
            conf_raw = v.get('rabbit_confidence', 1.0)
            stamp    = v.get('stamp', None)

            cx_param = self._row_val('center_x_px', 125.0)
            try:
                half_w = float(iw_raw) / 2.0 if iw_raw is not None else cx_param
            except (ValueError, TypeError):
                half_w = cx_param
            if half_w <= 0.0:
                half_w = cx_param

            deadband = self._row_val('chase_deadband', 0.02)
            lookahead = self._row_val('kf_feedforward_sec', 0.15)

            # Advance display-KF clock — we don't know commanded omega from here,
            # so predict with omega=0. This is OK for visualization: the FSM's
            # internal KF does the real prediction with the true omega input.
            import time
            now = time.monotonic()
            dt = 0.0 if self._last_dkf_time is None else (now - self._last_dkf_time)
            self._last_dkf_time = now
            self._dkf.predict(dt, 0.0)

            if visible and cx_raw is not None and stamp != self._last_vision_stamp:
                try:
                    self._dkf.update(float(cx_raw), float(conf_raw))
                except (ValueError, TypeError):
                    pass
                self._last_vision_stamp = stamp

            c = self._gcanvas
            w = c.winfo_width() or 500
            c.delete('all')
            mid = w // 2

            # Zero-line
            c.create_line(mid, 0, mid, 42, fill='#555555', dash=(3, 3))
            # Deadband band
            if deadband > 0.0:
                band_px = int((w // 2 - 4) * deadband)
                c.create_rectangle(mid - band_px, 4, mid + band_px, 38,
                                   fill='#2a2a2a', outline='')

            if visible and cx_raw is not None and self._dkf.initialized:
                try:
                    raw_cx = float(cx_raw)
                except (ValueError, TypeError):
                    raw_cx = cx_param

                # Raw YOLO position — dim grey dot
                raw_ratio = max(-1.0, min(1.0, (raw_cx - cx_param) / half_w))
                raw_x = int(mid + raw_ratio * (w // 2 - 4))
                c.create_oval(raw_x - 4, 6, raw_x + 4, 14, fill='#666666', outline='')

                # Filtered + lookahead position — bright bar
                pred_px = self._dkf.px + self._dkf.vx * lookahead
                err_norm = max(-1.5, min(1.5, (pred_px - cx_param) / half_w))
                ratio    = max(-1.0, min(1.0, err_norm))
                end      = int(mid + ratio * (w // 2 - 4))

                in_db = abs(err_norm) < deadband
                if in_db:
                    color = '#00cc44'
                elif abs(err_norm) < 0.4:
                    color = '#ffaa00'
                else:
                    color = '#cc2200'
                c.create_rectangle(min(mid, end), 18, max(mid, end), 34,
                                   fill=color, outline='')
                c.create_line(end, 16, end, 36, fill='white', width=2)

                # Labels
                self._lbl_err.config(text=f'err (KF) : {err_norm:+.3f}', fg=color)
                self._lbl_vx.config(text=f'vx : {self._dkf.vx:+6.0f} px/s',
                                    fg='#cccccc')
                self._lbl_vis.config(text='visible : YES', fg='#00cc44')
                self._lbl_state.config(
                    text='CENTERED' if in_db else 'TRACKING',
                    fg='#00cc44' if in_db else '#ffcc00')
            else:
                c.create_text(mid, 21, text='— no detection —',
                              fill='#444444', font=('', 9))
                self._lbl_err.config(text='err (KF) :  ---', fg='#555555')
                self._lbl_vx.config(text='vx : --- px/s', fg='#555555')
                self._lbl_vis.config(text='visible : NO', fg='#cc4444')
                self._lbl_state.config(text='')
        except Exception:
            pass
        self.root.after(50, self._refresh_gauge)   # 20 Hz display

    def reload(self) -> None:
        names   = [p[0] for p in PARAMS] + [c[0] for c in CHECKBOXES]
        current = self.client.get_current(names)
        for name, val in current.items():
            if name in self.rows:
                self.rows[name].set_value(float(val))
            elif name in self.bool_vars:
                self.bool_vars[name].set(bool(val))
        if self._lock_var.get():
            self._on_lock_toggle()
        self._reset_display_kf()

    def reset_defaults(self) -> None:
        for name, lo, hi, step, default, is_int, sec in PARAMS:
            if name in self.rows:
                self.rows[name].set_value(float(default), push=True)
        for name, default, sec in CHECKBOXES:
            if name in self.bool_vars:
                self.bool_vars[name].set(default)
                self.client.set_bool(name, default)
        if self._lock_var.get():
            self._on_lock_toggle()
        self._reset_display_kf()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    client    = ParamClient(TARGET_NODE)
    connected = client.wait_ready(timeout_sec=3.0)
    if not connected:
        print(f'WARNING: {TARGET_NODE} not reachable. '
              'Start wolf_fsm first, then relaunch for live values.')

    threading.Thread(target=client.spin_background, daemon=True).start()

    gui = TunerGUI(client, connected)
    try:
        gui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
