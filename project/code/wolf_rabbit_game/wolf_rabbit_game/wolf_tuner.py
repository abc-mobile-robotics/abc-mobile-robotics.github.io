#!/usr/bin/env python3
"""
Wolf FSM parameter tuner — slider + text entry edition.

Usage:
    python3 wolf_tuner.py

Requires wolf_fsm to have add_on_set_parameters_callback wired up.

Note: chase_kp/ki/kd now operate on NORMALIZED error (-1..+1 across the frame),
so the slider ranges here are very different from the old pixel-based gains.
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
#
# Gain ranges reflect that error is now normalized to roughly [-1, +1]:
#   kp = 2.0  →  rabbit at frame edge gives 2.0 rad/s turn command
#   kd = 0.15 →  moderate damping
#   ki rarely needed; add a whisker if there's steady-state offset
PARAMS: List[Tuple[str, float, float, float, float, bool, str]] = [
    ('chase_kp',               0.0,   8.0,   0.05,   2.0,    False, 'rotation'),
    ('chase_ki',               0.0,   2.0,   0.01,   0.0,    False, 'rotation'),
    ('chase_kd',               0.0,   1.0,   0.01,   0.15,   False, 'rotation'),
    ('chase_deadband',         0.0,   0.2,   0.005,  0.02,   False, 'rotation'),
    ('max_angular_speed',      0.2,   4.0,   0.05,   1.5,    False, 'rotation'),
    ('center_x_px',            50.0,  400.0, 1.0,    125.0,  False, 'rotation'),
    ('chase_linear_speed',     0.0,   2.5,   0.05,   0.0,    False, 'speed'),
    ('patrol_linear_speed',    0.0,   0.5,   0.01,   0.10,   False, 'speed'),
    ('patrol_turn_speed',      0.1,   1.5,   0.05,   0.65,   False, 'speed'),
    ('return_turn_speed',      0.1,   1.5,   0.05,   0.75,   False, 'speed'),
    ('vision_timeout_sec',     0.1,   2.0,   0.05,   0.7,    False, 'misc'),
    ('chase_lost_frames',      1,     20,    1,      5,      True,  'misc'),
]

CHECKBOXES: List[Tuple[str, bool, str]] = [
    ('chase_speed_scale_on_turn', True, 'speed'),
]

SECTION_COLORS = {
    'rotation': '#1a3a5c',
    'speed':    '#1a3a1a',
    'misc':     '#3a2a4a',
}
SECTION_LABELS = {
    'rotation': '  Rotation / Tracking',
    'speed':    '  Speed',
    'misc':     '  Misc',
}


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
        # Parse "key=value key=value …" format used by string_msg_to_dict
        d: dict = {}
        for token in msg.data.split():
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

    # setters
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

    # getter
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


# ── One parameter row ──────────────────────────────────────────────────────────

class ParamRow:
    """label | slider | [typed entry] | dot"""

    def __init__(self, parent: tk.Widget, client: ParamClient,
                 name: str, lo: float, hi: float, step: float,
                 initial: float, is_int: bool) -> None:
        self.client  = client
        self.name    = name
        self.lo      = lo
        self.hi      = hi
        self.step    = step
        self.is_int  = is_int
        self._guard  = False   # prevent slider↔entry feedback loops

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

    # ── helpers ────────────────────────────────────────────────────────────────

    def _fmt(self, v: float) -> str:
        if self.is_int:
            return str(int(round(v)))
        # Pick precision based on step so small gains (0.005) and
        # large values (150.0) both display cleanly.
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
            # flash red, restore last good value
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

    # ── public ─────────────────────────────────────────────────────────────────

    def set_value(self, v: float, push: bool = False) -> None:
        prev_guard = self._guard
        self._guard = True
        v = self._clamp(v)
        self.var.set(v)
        self.entry_sv.set(self._fmt(v))
        if push:
            self._push(v)
        self._guard = prev_guard   # restore — don't clear if caller set it

    def disable(self) -> None:
        # Disable BEFORE touching var so the slider command callback
        # cannot fire and race-push the old fetched value back to the node.
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

        # ── root ───────────────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title('Wolf FSM Tuner')
        self.root.geometry('700x880')
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

        # ── header ─────────────────────────────────────────────────────────────
        hdr = ttk.Frame(self.root, padding=(12, 8, 12, 4))
        hdr.pack(fill='x')
        tk.Label(hdr, text='Wolf FSM Tuner', fg='#e0e0e0', bg='#1e1e1e',
                 font=('', 13, 'bold')).pack(side='left')
        dot_color = '#00cc44' if connected else '#cc8800'
        dot_text  = f'● {TARGET_NODE}' if connected else '● offline — defaults shown'
        tk.Label(hdr, text=dot_text, fg=dot_color, bg='#1e1e1e',
                 font=('', 9)).pack(side='right')

        # ── rotation-only toggle ───────────────────────────────────────────────
        lock_bar = ttk.Frame(self.root, padding=(12, 2, 12, 4))
        lock_bar.pack(fill='x')
        self._lock_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            lock_bar,
            text='  Rotation-only mode  —  locks chase_linear_speed to 0  (great for PID tuning)',
            variable=self._lock_var,
            command=self._on_lock_toggle,
        ).pack(side='left')

        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(2, 0))

        # ── scrollable parameter area ──────────────────────────────────────────
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

        # ── fetch live values ──────────────────────────────────────────────────
        all_names = [p[0] for p in PARAMS] + [c[0] for c in CHECKBOXES]
        current = client.get_current(all_names) if connected else {}

        # ── build sections ─────────────────────────────────────────────────────
        for section in ('rotation', 'speed', 'misc'):
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

        # ── live gauge ─────────────────────────────────────────────────────────
        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(4, 0))
        self._build_gauge()

        # ── footer ─────────────────────────────────────────────────────────────
        footer = ttk.Frame(self.root, padding=(8, 6))
        footer.pack(fill='x', side='bottom')
        ttk.Button(footer, text='Reload from node',
                   command=self.reload).pack(side='left')
        ttk.Button(footer, text='Reset defaults',
                   command=self.reset_defaults).pack(side='left', padx=6)
        ttk.Button(footer, text='Quit',
                   command=self.root.quit).pack(side='right')

        # activate lock
        if self._lock_var.get():
            self._on_lock_toggle()

        # start gauge loop
        self._refresh_gauge()

    # ── section builder ────────────────────────────────────────────────────────

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

    # ── rotation-only lock ─────────────────────────────────────────────────────

    def _on_lock_toggle(self) -> None:
        locked = self._lock_var.get()
        row = self.rows.get('chase_linear_speed')
        if not isinstance(row, ParamRow):
            return
        if locked:
            # Disable FIRST (sets _guard=True) so slider command callback
            # cannot race and push the old fetched value back to the node.
            row.disable()
            row.set_value(0.0)          # update display only — guard is active
            self.client.set_double('chase_linear_speed', 0.0)  # explicit push
        else:
            row.enable()

    # ── live tracking gauge ────────────────────────────────────────────────────

    def _build_gauge(self) -> None:
        g = ttk.Frame(self.root, padding=(12, 6, 12, 4))
        g.pack(fill='x')

        tk.Label(g, text='Live tracking   /wolf/vision   (normalized error: -1 .. +1)',
                 fg='#888888', bg='#1e1e1e', font=('', 9, 'bold')).pack(anchor='w')

        self._gcanvas = tk.Canvas(g, height=30, bg='#242424',
                                  highlightthickness=1, highlightbackground='#444444')
        self._gcanvas.pack(fill='x', pady=(4, 4))

        row2 = tk.Frame(g, bg='#1e1e1e')
        row2.pack(fill='x')

        self._lbl_err  = tk.Label(row2, text='error :  ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=22, anchor='w')
        self._lbl_err.pack(side='left')
        self._lbl_vis  = tk.Label(row2, text='visible : ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=18, anchor='w')
        self._lbl_vis.pack(side='left')
        self._lbl_conf = tk.Label(row2, text='conf : ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=14, anchor='w')
        self._lbl_conf.pack(side='left')
        self._lbl_state = tk.Label(row2, text='',
                                   fg='#ffcc00', bg='#1e1e1e', font=('Courier', 9, 'bold'))
        self._lbl_state.pack(side='right')

    def _refresh_gauge(self) -> None:
        try:
            v        = self.client.get_vision()
            visible  = v.get('rabbit_visible', 'False').lower() == 'true'
            cx_raw   = v.get('rabbit_center_x', None)
            iw_raw   = v.get('image_width', None)
            conf_raw = v.get('rabbit_confidence', '0')

            # Resolve frame centre and half-width the SAME WAY the FSM does,
            # so what the gauge shows matches what the PID actually sees.
            cx_param = float(self.rows['center_x_px'].var.get()) \
                       if 'center_x_px' in self.rows else 125.0
            try:
                half_w = float(iw_raw) / 2.0 if iw_raw else cx_param
            except ValueError:
                half_w = cx_param
            if half_w <= 0.0:
                half_w = cx_param

            # Deadband for color thresholding (match FSM param if we have it)
            deadband = float(self.rows['chase_deadband'].var.get()) \
                       if 'chase_deadband' in self.rows else 0.02

            c   = self._gcanvas
            w   = c.winfo_width() or 500
            c.delete('all')
            # Zero-line
            c.create_line(w//2, 0, w//2, 30, fill='#555555', dash=(3, 3))
            # Deadband band
            if deadband > 0.0:
                band_px = int((w // 2 - 4) * deadband)
                c.create_rectangle(w//2 - band_px, 4, w//2 + band_px, 26,
                                   fill='#2a2a2a', outline='')

            if visible and cx_raw is not None:
                # Normalized error: px-from-center / half-width → -1..+1 at edges
                err_norm = (float(cx_raw) - cx_param) / half_w
                err_norm = max(-1.5, min(1.5, err_norm))
                conf     = float(conf_raw)

                mid   = w // 2
                ratio = max(-1.0, min(1.0, err_norm))
                end   = int(mid + ratio * (w // 2 - 4))

                in_db = abs(err_norm) < deadband
                if in_db:
                    color = '#00cc44'
                elif abs(err_norm) < 0.4:
                    color = '#ffaa00'
                else:
                    color = '#cc2200'

                c.create_rectangle(min(mid, end), 5, max(mid, end), 25,
                                   fill=color, outline='')
                c.create_line(end, 0, end, 30, fill='white', width=2)
                self._lbl_err.config(text=f'error : {err_norm:+.3f}', fg=color)
                self._lbl_conf.config(text=f'conf : {conf:.2f}', fg='#cccccc')
                self._lbl_vis.config(text='visible : YES', fg='#00cc44')
                self._lbl_state.config(
                    text='CENTERED' if in_db else 'TRACKING',
                    fg='#00cc44' if in_db else '#ffcc00')
            else:
                c.create_text(w//2, 15, text='— no detection —',
                              fill='#444444', font=('', 9))
                self._lbl_err.config(text='error :  ---', fg='#555555')
                self._lbl_conf.config(text='conf : ---',  fg='#555555')
                self._lbl_vis.config(text='visible : NO', fg='#cc4444')
                self._lbl_state.config(text='')
        except Exception:
            pass
        self.root.after(100, self._refresh_gauge)

    # ── toolbar actions ────────────────────────────────────────────────────────

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

    def run(self) -> None:
        self.root.mainloop()

#!/usr/bin/env python3
"""
Wolf FSM parameter tuner — slider + text entry edition.

Usage:
    python3 wolf_tuner.py

Requires wolf_fsm to have add_on_set_parameters_callback wired up.

Note: chase_kp/ki/kd now operate on NORMALIZED error (-1..+1 across the frame),
so the slider ranges here are very different from the old pixel-based gains.
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
#
# Gain ranges reflect that error is now normalized to roughly [-1, +1]:
#   kp = 2.0  →  rabbit at frame edge gives 2.0 rad/s turn command
#   kd = 0.15 →  moderate damping
#   ki rarely needed; add a whisker if there's steady-state offset
PARAMS: List[Tuple[str, float, float, float, float, bool, str]] = [
    ('chase_kp',               0.0,   8.0,   0.05,   2.0,    False, 'rotation'),
    ('chase_ki',               0.0,   2.0,   0.01,   0.0,    False, 'rotation'),
    ('chase_kd',               0.0,   1.0,   0.01,   0.15,   False, 'rotation'),
    ('chase_deadband',         0.0,   0.2,   0.005,  0.02,   False, 'rotation'),
    ('max_angular_speed',      0.2,   4.0,   0.05,   1.5,    False, 'rotation'),
    ('center_x_px',            50.0,  400.0, 1.0,    125.0,  False, 'rotation'),
    ('chase_linear_speed',     0.0,   2.5,   0.05,   0.0,    False, 'speed'),
    ('patrol_linear_speed',    0.0,   0.5,   0.01,   0.10,   False, 'speed'),
    ('patrol_turn_speed',      0.1,   1.5,   0.05,   0.65,   False, 'speed'),
    ('return_turn_speed',      0.1,   1.5,   0.05,   0.75,   False, 'speed'),
    ('vision_timeout_sec',     0.1,   2.0,   0.05,   0.7,    False, 'misc'),
    ('chase_lost_frames',      1,     20,    1,      5,      True,  'misc'),
]

CHECKBOXES: List[Tuple[str, bool, str]] = [
    ('chase_speed_scale_on_turn', True, 'speed'),
]

SECTION_COLORS = {
    'rotation': '#1a3a5c',
    'speed':    '#1a3a1a',
    'misc':     '#3a2a4a',
}
SECTION_LABELS = {
    'rotation': '  Rotation / Tracking',
    'speed':    '  Speed',
    'misc':     '  Misc',
}


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
        # Parse "key=value key=value …" format used by string_msg_to_dict
        d: dict = {}
        for token in msg.data.split():
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

    # setters
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

    # getter
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


# ── One parameter row ──────────────────────────────────────────────────────────

class ParamRow:
    """label | slider | [typed entry] | dot"""

    def __init__(self, parent: tk.Widget, client: ParamClient,
                 name: str, lo: float, hi: float, step: float,
                 initial: float, is_int: bool) -> None:
        self.client  = client
        self.name    = name
        self.lo      = lo
        self.hi      = hi
        self.step    = step
        self.is_int  = is_int
        self._guard  = False   # prevent slider↔entry feedback loops

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

    # ── helpers ────────────────────────────────────────────────────────────────

    def _fmt(self, v: float) -> str:
        if self.is_int:
            return str(int(round(v)))
        # Pick precision based on step so small gains (0.005) and
        # large values (150.0) both display cleanly.
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
            # flash red, restore last good value
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

    # ── public ─────────────────────────────────────────────────────────────────

    def set_value(self, v: float, push: bool = False) -> None:
        prev_guard = self._guard
        self._guard = True
        v = self._clamp(v)
        self.var.set(v)
        self.entry_sv.set(self._fmt(v))
        if push:
            self._push(v)
        self._guard = prev_guard   # restore — don't clear if caller set it

    def disable(self) -> None:
        # Disable BEFORE touching var so the slider command callback
        # cannot fire and race-push the old fetched value back to the node.
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

        # ── root ───────────────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title('Wolf FSM Tuner')
        self.root.geometry('700x880')
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

        # ── header ─────────────────────────────────────────────────────────────
        hdr = ttk.Frame(self.root, padding=(12, 8, 12, 4))
        hdr.pack(fill='x')
        tk.Label(hdr, text='Wolf FSM Tuner', fg='#e0e0e0', bg='#1e1e1e',
                 font=('', 13, 'bold')).pack(side='left')
        dot_color = '#00cc44' if connected else '#cc8800'
        dot_text  = f'● {TARGET_NODE}' if connected else '● offline — defaults shown'
        tk.Label(hdr, text=dot_text, fg=dot_color, bg='#1e1e1e',
                 font=('', 9)).pack(side='right')

        # ── rotation-only toggle ───────────────────────────────────────────────
        lock_bar = ttk.Frame(self.root, padding=(12, 2, 12, 4))
        lock_bar.pack(fill='x')
        self._lock_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            lock_bar,
            text='  Rotation-only mode  —  locks chase_linear_speed to 0  (great for PID tuning)',
            variable=self._lock_var,
            command=self._on_lock_toggle,
        ).pack(side='left')

        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(2, 0))

        # ── scrollable parameter area ──────────────────────────────────────────
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

        # ── fetch live values ──────────────────────────────────────────────────
        all_names = [p[0] for p in PARAMS] + [c[0] for c in CHECKBOXES]
        current = client.get_current(all_names) if connected else {}

        # ── build sections ─────────────────────────────────────────────────────
        for section in ('rotation', 'speed', 'misc'):
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

        # ── live gauge ─────────────────────────────────────────────────────────
        tk.Frame(self.root, bg='#333333', height=1).pack(fill='x', padx=8, pady=(4, 0))
        self._build_gauge()

        # ── footer ─────────────────────────────────────────────────────────────
        footer = ttk.Frame(self.root, padding=(8, 6))
        footer.pack(fill='x', side='bottom')
        ttk.Button(footer, text='Reload from node',
                   command=self.reload).pack(side='left')
        ttk.Button(footer, text='Reset defaults',
                   command=self.reset_defaults).pack(side='left', padx=6)
        ttk.Button(footer, text='Quit',
                   command=self.root.quit).pack(side='right')

        # activate lock
        if self._lock_var.get():
            self._on_lock_toggle()

        # start gauge loop
        self._refresh_gauge()

    # ── section builder ────────────────────────────────────────────────────────

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

    # ── rotation-only lock ─────────────────────────────────────────────────────

    def _on_lock_toggle(self) -> None:
        locked = self._lock_var.get()
        row = self.rows.get('chase_linear_speed')
        if not isinstance(row, ParamRow):
            return
        if locked:
            # Disable FIRST (sets _guard=True) so slider command callback
            # cannot race and push the old fetched value back to the node.
            row.disable()
            row.set_value(0.0)          # update display only — guard is active
            self.client.set_double('chase_linear_speed', 0.0)  # explicit push
        else:
            row.enable()

    # ── live tracking gauge ────────────────────────────────────────────────────

    def _build_gauge(self) -> None:
        g = ttk.Frame(self.root, padding=(12, 6, 12, 4))
        g.pack(fill='x')

        tk.Label(g, text='Live tracking   /wolf/vision   (normalized error: -1 .. +1)',
                 fg='#888888', bg='#1e1e1e', font=('', 9, 'bold')).pack(anchor='w')

        self._gcanvas = tk.Canvas(g, height=30, bg='#242424',
                                  highlightthickness=1, highlightbackground='#444444')
        self._gcanvas.pack(fill='x', pady=(4, 4))

        row2 = tk.Frame(g, bg='#1e1e1e')
        row2.pack(fill='x')

        self._lbl_err  = tk.Label(row2, text='error :  ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=22, anchor='w')
        self._lbl_err.pack(side='left')
        self._lbl_vis  = tk.Label(row2, text='visible : ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=18, anchor='w')
        self._lbl_vis.pack(side='left')
        self._lbl_conf = tk.Label(row2, text='conf : ---',
                                  fg='#888888', bg='#1e1e1e', font=('Courier', 9), width=14, anchor='w')
        self._lbl_conf.pack(side='left')
        self._lbl_state = tk.Label(row2, text='',
                                   fg='#ffcc00', bg='#1e1e1e', font=('Courier', 9, 'bold'))
        self._lbl_state.pack(side='right')

    def _refresh_gauge(self) -> None:
        try:
            v        = self.client.get_vision()
            visible  = v.get('rabbit_visible', 'False').lower() == 'true'
            cx_raw   = v.get('rabbit_center_x', None)
            iw_raw   = v.get('image_width', None)
            conf_raw = v.get('rabbit_confidence', '0')

            # Resolve frame centre and half-width the SAME WAY the FSM does,
            # so what the gauge shows matches what the PID actually sees.
            cx_param = float(self.rows['center_x_px'].var.get()) \
                       if 'center_x_px' in self.rows else 125.0
            try:
                half_w = float(iw_raw) / 2.0 if iw_raw else cx_param
            except ValueError:
                half_w = cx_param
            if half_w <= 0.0:
                half_w = cx_param

            # Deadband for color thresholding (match FSM param if we have it)
            deadband = float(self.rows['chase_deadband'].var.get()) \
                       if 'chase_deadband' in self.rows else 0.02

            c   = self._gcanvas
            w   = c.winfo_width() or 500
            c.delete('all')
            # Zero-line
            c.create_line(w//2, 0, w//2, 30, fill='#555555', dash=(3, 3))
            # Deadband band
            if deadband > 0.0:
                band_px = int((w // 2 - 4) * deadband)
                c.create_rectangle(w//2 - band_px, 4, w//2 + band_px, 26,
                                   fill='#2a2a2a', outline='')

            if visible and cx_raw is not None:
                # Normalized error: px-from-center / half-width → -1..+1 at edges
                err_norm = (float(cx_raw) - cx_param) / half_w
                err_norm = max(-1.5, min(1.5, err_norm))
                conf     = float(conf_raw)

                mid   = w // 2
                ratio = max(-1.0, min(1.0, err_norm))
                end   = int(mid + ratio * (w // 2 - 4))

                in_db = abs(err_norm) < deadband
                if in_db:
                    color = '#00cc44'
                elif abs(err_norm) < 0.4:
                    color = '#ffaa00'
                else:
                    color = '#cc2200'

                c.create_rectangle(min(mid, end), 5, max(mid, end), 25,
                                   fill=color, outline='')
                c.create_line(end, 0, end, 30, fill='white', width=2)
                self._lbl_err.config(text=f'error : {err_norm:+.3f}', fg=color)
                self._lbl_conf.config(text=f'conf : {conf:.2f}', fg='#cccccc')
                self._lbl_vis.config(text='visible : YES', fg='#00cc44')
                self._lbl_state.config(
                    text='CENTERED' if in_db else 'TRACKING',
                    fg='#00cc44' if in_db else '#ffcc00')
            else:
                c.create_text(w//2, 15, text='— no detection —',
                              fill='#444444', font=('', 9))
                self._lbl_err.config(text='error :  ---', fg='#555555')
                self._lbl_conf.config(text='conf : ---',  fg='#555555')
                self._lbl_vis.config(text='visible : NO', fg='#cc4444')
                self._lbl_state.config(text='')
        except Exception:
            pass
        self.root.after(100, self._refresh_gauge)

    # ── toolbar actions ────────────────────────────────────────────────────────

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

    def run(self) -> None:
        self.root.mainloop()


# ── entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    client    = ParamClient(TARGET_NODE)
    connected = client.wait_ready(timeout_sec=3.0)
    if not connected:
        print(f'WARNING: {TARGET_NODE} not reachable. '
              'Start wolf_fsm first, then relaunch for live values.')

    # spin ROS in background — keeps subscriptions and service calls alive
    threading.Thread(target=client.spin_background, daemon=True).start()

    gui = TunerGUI(client, connected)
    try:
        gui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    threading.Thread(target=client.spin_background, daemon=True).start()

    gui = TunerGUI(client, connected)
    try:
        gui.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
