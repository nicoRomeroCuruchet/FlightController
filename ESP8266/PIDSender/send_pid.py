#!/usr/bin/env python3
"""
send_pid.py — interactive PID writer for the ESP8266 / blackpill bridge.

Usage:
    python3 send_pid.py [--host 192.168.0.12] [--port 8888]

Flujo:
    - Muestra los valores por defecto (los hardcoded en main.c).
    - Te prompteea por cada gain (ENTER mantiene el actual).
    - Manda el CSV al ESP.
    - El ESP forward al blackpill por UART, y echo "OK ..." de vuelta.

Recordatorio:
    El blackpill aplica los gains nuevos sólo cuando está DISARMED.
    Si PC13 no se prende, chequeá que el quad esté disarmed.
"""

import argparse
import socket
import sys

DEFAULT_HOST = "192.168.0.10"
DEFAULT_PORT = 8888

# Defaults match main.c (líneas 361-371): F450 / 1400kv / 10x4.7 starting points
DEFAULT_GAINS = {
    "roll":  {"kp": 6.0, "ki": 0.0, "kd": 3.0},
    "pitch": {"kp": 6.0, "ki": 0.0, "kd": 3.0},
    "yaw":   {"kp": 3.0, "ki": 0.0, "kd": 8.0},
}
# Filtro complementario gyro<->DMP: peso del DMP por update (~200 Hz).
# 0.04 ≈ τ=125ms. Subir = más DMP (corrige drift, más sensible a vibración).
DEFAULT_COMP_BETA = 0.04

AXES = ("roll", "pitch", "yaw")
GAINS = ("kp", "ki", "kd")


def prompt_float(label: str, current: float) -> float:
    while True:
        raw = input(f"  {label} [{current}]: ").strip()
        if raw == "":
            return current
        try:
            return float(raw)
        except ValueError:
            print(f"  '{raw}' no es un número, probá de nuevo.")


def prompt_gains(current: dict, current_beta: float) -> tuple:
    print("\nIngresá los nuevos valores (ENTER para mantener el actual):")
    new = {}
    for axis in AXES:
        print(f" [{axis.upper()}]")
        new[axis] = {}
        for g in GAINS:
            new[axis][g] = prompt_float(g, current[axis][g])
    print(" [Filtro complementario]")
    new_beta = prompt_float("comp_beta (0.001-0.5)", current_beta)
    return new, new_beta


def gains_to_csv(g: dict, beta: float) -> str:
    vals = []
    for axis in AXES:
        for k in GAINS:
            vals.append(f"{g[axis][k]:.6f}")
    vals.append(f"{beta:.6f}")
    return ",".join(vals)


def send_once(host: str, port: int, gains: dict, beta: float, timeout: float = 3.0) -> str:
    line = gains_to_csv(gains, beta) + "\n"
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(line.encode("ascii"))
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(256)
            if not chunk:
                break
            buf += chunk
    return buf.decode("ascii", errors="replace").strip()


def show_gains(gains: dict, beta: float) -> None:
    for axis in AXES:
        g = gains[axis]
        print(f"  {axis:5s}  Kp={g['kp']:>8.4f}  Ki={g['ki']:>8.4f}  Kd={g['kd']:>8.4f}")
    print(f"  comp_beta = {beta:.4f}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST,
                   help=f"ESP IP address (default: {DEFAULT_HOST})")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help=f"ESP TCP port (default: {DEFAULT_PORT})")
    args = p.parse_args()

    print(f"Target: {args.host}:{args.port}")
    print("Valores por defecto (los hardcoded en main.c):")
    show_gains(DEFAULT_GAINS, DEFAULT_COMP_BETA)

    current = {axis: dict(DEFAULT_GAINS[axis]) for axis in AXES}
    current_beta = DEFAULT_COMP_BETA

    while True:
        try:
            current, current_beta = prompt_gains(current, current_beta)
        except (EOFError, KeyboardInterrupt):
            print("\nSaliendo.")
            return 0

        print("\nMandando:")
        show_gains(current, current_beta)

        try:
            reply = send_once(args.host, args.port, current, current_beta)
        except (socket.timeout, ConnectionError, OSError) as e:
            print(f"  Error de socket: {e}")
            continue

        if reply.startswith("OK"):
            print(f"  ESP respondió: {reply}")
            print("  (verificá PC13 del blackpill: debe haberse encendido)")
        else:
            print(f"  ESP respondió: {reply!r}")

        try:
            cont = input("\n¿Mandar otro? [Y/n]: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if cont in ("n", "no"):
            return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
