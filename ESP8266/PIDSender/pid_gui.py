#!/usr/bin/env python3
"""
pid_gui.py — interactive Tk GUI for tuning PIDs on the ESP8266/blackpill bridge.

Usage:
    python3 pid_gui.py [--host 192.168.0.12] [--port 8888]

Una grilla 3x3 de sliders (filas: roll/pitch/yaw, columnas: Kp/Ki/Kd).
Movés las barritas con el mouse o las flechas del teclado, y cuando estás
listo presionás "Enviar" para mandar los 9 valores al blackpill.

Recordatorio:
    El blackpill aplica los gains nuevos sólo cuando está DISARMED.
    Si PC13 no se prende, chequeá que el quad esté disarmed.
"""

import argparse
import socket
import sys
import tkinter as tk
from tkinter import ttk

DEFAULT_HOST = "192.168.0.10"
DEFAULT_PORT = 8888

# Defaults match main.c (líneas 361-371): F450 / 1400kv / 10x4.7 starting points
DEFAULT_GAINS = {
    "roll":  {"kp": 6.0, "ki": 0.0, "kd": 3.0},
    "pitch": {"kp": 6.0, "ki": 0.0, "kd": 3.0},
    "yaw":   {"kp": 3.0, "ki": 0.0, "kd": 8.0},
}
DEFAULT_COMP_BETA = 0.04   # peso del DMP en el filtro complementario

AXES = ("roll", "pitch", "yaw")
GAINS = ("kp", "ki", "kd")

# Rango por tipo de gain
RANGES = {
    "kp": (0.0, 30.0),
    "ki": (0.0,  5.0),
    "kd": (0.0, 20.0),
}
RESOLUTION = 0.1
SLIDER_LENGTH = 180

# Filtro complementario: slider más fino, rango chico
COMP_BETA_RANGE = (0.001, 0.2)
COMP_BETA_RESOLUTION = 0.001
COMP_BETA_SLIDER_LENGTH = 400

SOCKET_TIMEOUT_S = 3.0


class PIDTuner:
    def __init__(self, root, host, port):
        self.root = root
        self.host = host
        self.port = port

        # Dict[axis][gain] -> (DoubleVar, value_label)
        self.cells = {}

        root.title(f"PID Tuner — {host}:{port}")
        root.minsize(720, 440)

        self._build_ui()
        self._load_defaults()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=15)
        main.pack(fill="both", expand=True)

        title = ttk.Label(main, text="PID Gains",
                          font=("TkDefaultFont", 14, "bold"))
        title.grid(row=0, column=0, columnspan=4, pady=(0, 12))

        # Header: empty corner + Kp/Ki/Kd labels
        for c, gain in enumerate(GAINS, start=1):
            lo, hi = RANGES[gain]
            txt = f"{gain.upper()}\n({lo:g}–{hi:g})"
            ttk.Label(main, text=txt, anchor="center",
                      font=("TkDefaultFont", 10, "bold"),
                      justify="center").grid(row=1, column=c, padx=10, pady=(0, 8))

        # Slider rows: one per axis
        for r, axis in enumerate(AXES, start=2):
            ttk.Label(main, text=axis.upper(),
                      font=("TkDefaultFont", 11, "bold")).grid(
                row=r, column=0, sticky="w", padx=(0, 10))

            self.cells[axis] = {}
            for c, gain in enumerate(GAINS, start=1):
                lo, hi = RANGES[gain]
                var = tk.DoubleVar(value=0.0)

                cell = ttk.Frame(main)
                cell.grid(row=r, column=c, padx=10, pady=6)

                value_label = ttk.Label(cell, text="0.00", width=7,
                                        anchor="center",
                                        font=("TkFixedFont", 10))
                value_label.pack()

                def on_change(v, lbl=value_label):
                    try:
                        lbl.config(text=f"{float(v):.2f}")
                    except ValueError:
                        pass

                slider = tk.Scale(cell,
                                  from_=lo, to=hi,
                                  resolution=RESOLUTION,
                                  orient="horizontal",
                                  length=SLIDER_LENGTH,
                                  variable=var,
                                  showvalue=0,
                                  command=on_change)
                slider.pack()

                self.cells[axis][gain] = (var, value_label)

        # ----- Filtro complementario (fila aparte) -----
        sep = ttk.Separator(main, orient="horizontal")
        sep.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(12, 8))

        comp_frame = ttk.Frame(main)
        comp_frame.grid(row=6, column=0, columnspan=4, sticky="ew", padx=4)

        ttk.Label(comp_frame, text="Comp β",
                  font=("TkDefaultFont", 11, "bold")).pack(side="left", padx=(0, 12))

        self.beta_var = tk.DoubleVar(value=DEFAULT_COMP_BETA)
        self.beta_label = ttk.Label(comp_frame,
                                    text=f"{DEFAULT_COMP_BETA:.3f}",
                                    width=8, anchor="center",
                                    font=("TkFixedFont", 10))
        self.beta_label.pack(side="left", padx=(0, 8))

        def on_beta_change(v):
            try:
                self.beta_label.config(text=f"{float(v):.3f}")
            except ValueError:
                pass

        beta_slider = tk.Scale(comp_frame,
                               from_=COMP_BETA_RANGE[0],
                               to=COMP_BETA_RANGE[1],
                               resolution=COMP_BETA_RESOLUTION,
                               orient="horizontal",
                               length=COMP_BETA_SLIDER_LENGTH,
                               variable=self.beta_var,
                               showvalue=0,
                               command=on_beta_change)
        beta_slider.pack(side="left", fill="x", expand=True)

        ttk.Label(comp_frame,
                  text=f"({COMP_BETA_RANGE[0]:g}–{COMP_BETA_RANGE[1]:g})",
                  font=("TkDefaultFont", 9)).pack(side="left", padx=(8, 0))

        # Botones
        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=7, column=0, columnspan=4, pady=(15, 5))

        ttk.Button(btn_frame, text="Cargar defaults",
                   command=self._load_defaults).pack(side="left", padx=8)
        ttk.Button(btn_frame, text="Cero todo",
                   command=self._zero_all).pack(side="left", padx=8)
        send_btn = ttk.Button(btn_frame, text="Enviar",
                              command=self._send)
        send_btn.pack(side="left", padx=8)

        # ENTER también envía
        self.root.bind("<Return>", lambda _e: self._send())

        # Status bar
        self.status = ttk.Label(main,
                                text="Listo. Mové las barritas y presioná Enviar (o ENTER).",
                                relief="sunken", anchor="w",
                                padding=(6, 4))
        self.status.grid(row=8, column=0, columnspan=4, sticky="ew",
                         pady=(10, 0))

        # Hacer que las columnas se expandan
        for c in range(1, 4):
            main.columnconfigure(c, weight=1)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def _load_defaults(self):
        for axis in AXES:
            for gain in GAINS:
                v, lbl = self.cells[axis][gain]
                val = DEFAULT_GAINS[axis][gain]
                v.set(val)
                lbl.config(text=f"{val:.2f}")
        self.beta_var.set(DEFAULT_COMP_BETA)
        self.beta_label.config(text=f"{DEFAULT_COMP_BETA:.3f}")
        self._set_status(
            "Defaults cargados. Presioná Enviar para escribirlos al blackpill.")

    def _zero_all(self):
        for axis in AXES:
            for gain in GAINS:
                v, lbl = self.cells[axis][gain]
                v.set(0.0)
                lbl.config(text="0.00")
        # beta NO se va a 0 porque rompería el filtro complementario.
        # Mantenemos el valor actual.
        self._set_status("Todos los gains a 0 (β del filtro intacto). "
                         "Cuidado: con gains en 0 no hay control.")

    def _gains_csv(self) -> str:
        parts = []
        for axis in AXES:
            for k in GAINS:
                parts.append(f"{self.cells[axis][k][0].get():.6f}")
        parts.append(f"{self.beta_var.get():.6f}")
        return ",".join(parts)

    def _send(self):
        csv = self._gains_csv() + "\n"
        self._set_status(f"Enviando a {self.host}:{self.port}...")
        self.root.update_idletasks()

        try:
            with socket.create_connection((self.host, self.port),
                                          timeout=SOCKET_TIMEOUT_S) as s:
                s.sendall(csv.encode("ascii"))
                buf = b""
                while not buf.endswith(b"\n"):
                    chunk = s.recv(256)
                    if not chunk:
                        break
                    buf += chunk
            reply = buf.decode("ascii", errors="replace").strip()
        except (socket.timeout, ConnectionError, OSError) as e:
            self._set_status(f"Error de socket: {e}", error=True)
            return

        if reply.startswith("OK"):
            self._set_status(
                f"Enviado OK. ESP echo: {reply[:80]}  "
                f"(chequeá PC13 — si está disarmed se prende).")
        else:
            self._set_status(f"Respuesta inesperada: {reply!r}", error=True)

    def _set_status(self, msg: str, error: bool = False):
        self.status.config(text=msg, foreground=("red" if error else "black"))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST,
                   help=f"ESP IP (default: {DEFAULT_HOST})")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help=f"ESP TCP port (default: {DEFAULT_PORT})")
    args = p.parse_args()

    root = tk.Tk()
    PIDTuner(root, args.host, args.port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
