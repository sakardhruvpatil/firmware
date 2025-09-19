#!/usr/bin/env python3
"""
Speed logger for NUCLEO-F446ZE motor firmware.
Reads lines like:
  Pulses:123 Hz:45.67 RPM:228.3 Target%:70.0 Curr%:52.4
or with " (NO SIGNAL)" present.

Usage:
  python3 tools/speed_logger.py --port /dev/ttyACM0 --baud 115200 --csv speed_log.csv

Press Ctrl+C to stop. If PlatformIO serial monitor is open, close it first.
"""
import argparse
import csv
import datetime as dt
import re
import sys
import time

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is not installed. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Regex to capture values regardless of spacing and optional (NO SIGNAL)
LINE_RE = re.compile(
    r"Pulses:(?P<pulses>\d+)\s+Hz:(?P<hz>[\d\.\-]+)\s+RPM:(?P<rpm>[\d\.\-]+)(?:\s+\(NO SIGNAL\))?\s+Target%:(?P<target>[\d\.\-]+)\s+Curr%:(?P<curr>[\d\.\-]+)"
)

def parse_args():
    p = argparse.ArgumentParser(description="Read speed/RPM lines from serial and optionally log to CSV")
    p.add_argument("--port", required=False, help="Serial port, e.g. /dev/ttyACM0 (Linux) or COM3 (Windows)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--csv", default=None, help="CSV file to write (optional)")
    p.add_argument("--list", action="store_true", help="List available ports and exit")
    p.add_argument("--silent", action="store_true", help="Don't print to console, only CSV")
    return p.parse_args()


def list_serial_ports():
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"{p.device} - {p.description}")


def open_serial(port: str | None, baud: int) -> serial.Serial:
    if port is None:
        # Try to auto-pick a likely port on Linux: '/dev/ttyACM*' or '/dev/ttyUSB*'
        candidates = [p.device for p in list_ports.comports() if ("ACM" in p.device or "USB" in p.device)]
        if not candidates:
            raise RuntimeError("No port specified and no likely ports found. Use --list and --port.")
        port = candidates[0]
        print(f"Auto-selected port: {port}")
    ser = serial.Serial(port=port, baudrate=baud, timeout=1)
    # Request exclusive access on POSIX (prevents other apps from opening simultaneously)
    try:
        # pyserial exposes 'exclusive' on posix; ignore if unsupported
        ser.exclusive = True  # type: ignore[attr-defined]
    except Exception:
        pass
    # Give MCU a moment after opening port
    time.sleep(0.3)
    # Flush any boot noise
    ser.reset_input_buffer()
    return ser


def main():
    args = parse_args()

    if args.list:
        list_serial_ports()
        return

    try:
        ser = open_serial(args.port, args.baud)
    except Exception as e:
        print(f"Failed to open serial: {e}", file=sys.stderr)
        sys.exit(2)

    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp_iso", "pulses", "hz", "rpm", "target_percent", "current_percent", "raw_line"])  # header

    print("Reading... Press Ctrl+C to stop.")

    try:
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                print("Serial error: port disconnected or in use by another application (e.g., PlatformIO Monitor).", file=sys.stderr)
                print(f"Details: {e}", file=sys.stderr)
                break
            if not line:
                continue
            m = LINE_RE.search(line)
            if not m:
                # Uncomment for debugging unknown format lines
                # print(f"Unparsed: {line}")
                continue
            ts = dt.datetime.now(dt.timezone.utc).isoformat()
            pulses = int(m.group("pulses"))
            hz = float(m.group("hz"))
            rpm = float(m.group("rpm"))
            target = float(m.group("target"))
            curr = float(m.group("curr"))

            if not args.silent:
                print(f"{ts} | RPM: {rpm:8.2f} | Hz: {hz:7.2f} | Pulses: {pulses:8d} | Target%: {target:6.1f} | Curr%: {curr:6.1f}")

            if csv_writer:
                csv_writer.writerow([ts, pulses, hz, rpm, target, curr, line])
                # Optionally flush periodically for safety
                # csv_file.flush()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            if csv_file:
                csv_file.flush()
                csv_file.close()
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()
