#!/usr/bin/env python3
"""Record one trial from the ElderCare board's CSV data log.

The board must run the data-logging firmware (DATA_LOG_MODE 1 in
data_logger.h). Close any serial terminal first: two programs reading the
same port each get only part of the data.

    python3 capture.py sit_heavy               -> recordings/sit_heavy_01.csv, _02, ...
    python3 capture.py fall_forward -n "bounced off cushion"
    python3 capture.py still_flat -s 30        -> stops by itself after 30 s

Press the board's black RESET button when asked. Everything before the
board's '# ElderCare data log' header is discarded, so lines the ST-LINK
buffered from an earlier run never end up in a recording. Hold the board
still until GO so the detector can set its reference posture, then perform
the motion. Recording runs until Ctrl-C (or until -s seconds, if given), and
everything recorded is saved.

Standard library only (termios), so it runs on Linux and macOS without
installing anything.
"""

import argparse
import glob
import os
import re
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
DETECTOR_HEADER = HERE.parent / "Assignment/CG2028_Assignment/Core/Inc/fall_detector.h"

LOG_START = "# ElderCare data log"
N_COLUMNS = 16                     # t_ms, 3 accel, 3 gyro, 3 + 3 filtered, phase, event, asm_ok
T_COL, PHASE_COL, EVENT_COL, ASM_COL = 0, 13, 14, 15
HEADER_TIMEOUT_S = 30.0            # time allowed for pressing reset
NO_DATA_TIMEOUT_S = 2.0            # the board sends 50 lines/s, so 2 s of silence is a fault


def enum_names(type_name, prefix):
    """Enum member names from fall_detector.h, so labels follow the C code."""
    try:
        text = DETECTOR_HEADER.read_text()
    except OSError:
        return {}
    match = re.search(r"typedef enum\s*{([^}]*)}\s*" + type_name + r"\s*;", text)
    if not match:
        return {}
    body = re.sub(r"/\*.*?\*/", "", match.group(1), flags=re.S)
    names = [name.strip() for name in body.split(",") if name.strip()]
    return {value: name.removeprefix(prefix) for value, name in enumerate(names)}


PHASES = enum_names("FallPhase", "FALL_PHASE_")
EVENTS = enum_names("FallEvent", "FALL_EVENT_")


def default_port():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


def open_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[4] = attrs[5] = termios.B115200   # the ST-LINK uses the host's baud rate
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def read_lines(fd, timeout_s):
    """Yield decoded lines as they arrive; yield None after timeout_s of silence."""
    pending = b""
    while True:
        ready, _, _ = select.select([fd], [], [], timeout_s)
        if not ready:
            yield None
            continue
        chunk = os.read(fd, 4096)
        if not chunk:
            return
        pending += chunk
        *lines, pending = pending.split(b"\n")
        for line in lines:
            yield line.rstrip(b"\r").decode("ascii", errors="replace")


def parse_row(line):
    fields = line.split(",")
    if len(fields) != N_COLUMNS:
        return None
    try:
        return [int(field) for field in fields]
    except ValueError:
        return None


def output_path(out_dir, name, force):
    """name_01.csv, name_02.csv, ... unless the name already ends in a number."""
    if re.search(r"_\d+$", name):
        path = out_dir / f"{name}.csv"
        if path.exists() and not force:
            sys.exit(f"{path} already exists; pick another name or use --force.")
        return path
    for number in range(1, 1000):
        path = out_dir / f"{name}_{number:02d}.csv"
        if not path.exists():
            return path
    sys.exit(f"Too many recordings named {name}_NN.csv.")


def status(text):
    """Print a message on its own line, clearing the progress line first."""
    print(f"\r{text:<60}")


def wait_for_header(lines):
    print("Press the black RESET button on the board now...")
    deadline = time.monotonic() + HEADER_TIMEOUT_S
    for line in lines:
        if line is None:
            if time.monotonic() > deadline:
                sys.exit(f"No '{LOG_START}' header within {HEADER_TIMEOUT_S:.0f} s. Is the "
                         "logging firmware (DATA_LOG_MODE 1) flashed and running, and no "
                         "other program reading the port?")
            continue
        # Not startswith: a reset mid-line leaves that line unterminated, so
        # the header arrives glued to its end (and reset can add a stray byte).
        if LOG_START in line:
            header = [LOG_START]
            break
    else:
        sys.exit("Serial port closed while waiting for the header.")

    # Remaining '#' lines, then the column names.
    for line in lines:
        if line is None:
            sys.exit("The board stopped sending in the middle of its header.")
        header.append(line)
        if not line.startswith("#"):
            return header
    sys.exit("Serial port closed in the middle of the header.")


def record(lines, seconds, settle_s):
    """Collect data rows until Ctrl-C, or until `seconds` of board time if given."""
    rows, texts = [], []
    malformed = 0
    last_whole_second = -1
    said_go = False
    try:
        for line in lines:
            if line is None:
                status(f"!! No data for {NO_DATA_TIMEOUT_S:.0f} s; stopping.")
                break
            row = parse_row(line)
            if row is None:
                malformed += 1
                continue
            if rows and row[T_COL] < rows[-1][T_COL]:
                status("!! Timestamps went backwards: the board was reset mid-trial. Stopping.")
                break
            rows.append(row)
            texts.append(line)

            elapsed = (row[T_COL] - rows[0][T_COL]) / 1000.0
            if row[EVENT_COL] != 0:
                status(f"  {elapsed:5.2f} s  {EVENTS.get(row[EVENT_COL], row[EVENT_COL])}")
            if not said_go and elapsed >= settle_s:
                status("\a  GO - perform the motion now")
                said_go = True
            if int(elapsed) != last_whole_second:
                last_whole_second = int(elapsed)
                limit = f" / {seconds:g} s" if seconds is not None else " s  (Ctrl-C to stop)"
                print(f"\r  {last_whole_second:3d}{limit}", end="", flush=True)
            if seconds is not None and elapsed >= seconds:
                break
    except KeyboardInterrupt:
        status("  Stopped (Ctrl-C).")
    print()
    return rows, texts, malformed


def summarise(rows, header, malformed):
    period = re.search(r"sample_period_ms=(\d+)", "\n".join(header))
    period_ms = int(period.group(1)) if period else None

    duration = (rows[-1][T_COL] - rows[0][T_COL]) / 1000.0
    steps = [b[T_COL] - a[T_COL] for a, b in zip(rows, rows[1:])]
    off_period = [step for step in steps if period_ms and step != period_ms]
    mismatches = sum(1 for row in rows if row[ASM_COL] != 1)
    events = [row[EVENT_COL] for row in rows if row[EVENT_COL] != 0]
    ready = 1 in events                   # FALL_EVENT_READY

    print(f"  {len(rows)} samples over {duration:.2f} s")
    if off_period:
        print(f"  !! {len(off_period)} of {len(steps)} steps were not {period_ms} ms "
              f"(largest {max(off_period)} ms)")
    if malformed:
        print(f"  !! {malformed} malformed line(s) dropped")
    if mismatches:
        print(f"  !! assembly and C EWMA differed on {mismatches} sample(s)")
    if not ready:
        print("  !! No READY event: the reference posture was never set. Hold the board "
              "still after reset.")
    print(f"  Final phase: {PHASES.get(rows[-1][PHASE_COL], rows[-1][PHASE_COL])}")


def main():
    parser = argparse.ArgumentParser(
        description="Record one trial from the board's CSV data log.")
    parser.add_argument("name", help="activity name, e.g. sit_heavy or fall_forward")
    parser.add_argument("-s", "--seconds", type=float, default=None,
                        help="stop after this many seconds (default: record until Ctrl-C)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds to hold still before GO (default 2)")
    parser.add_argument("-n", "--note", help="free-text note stored in the file")
    parser.add_argument("-p", "--port", default=default_port(),
                        help="serial port (default: first /dev/ttyACM* or /dev/cu.usbmodem*)")
    parser.add_argument("-o", "--out-dir", type=Path, default=HERE / "recordings",
                        help="output folder (default: recordings/ next to this script)")
    parser.add_argument("--force", action="store_true",
                        help="overwrite an existing file with an explicit number")
    args = parser.parse_args()

    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        sys.exit("Use letters, digits, '_' and '-' only in the name.")
    if args.port is None:
        sys.exit("No board found. Plug it in, or pass --port.")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    path = output_path(args.out_dir, args.name, args.force)

    try:
        fd = open_port(args.port)
    except OSError as error:
        sys.exit(f"Cannot open {args.port}: {error}")
    try:
        lines = read_lines(fd, NO_DATA_TIMEOUT_S)
        header = wait_for_header(lines)
        length = f"{args.seconds:g} s" if args.seconds is not None else "until Ctrl-C"
        print(f"Recording {length} to {path.name}. "
              f"Hold still for {args.settle:g} s until GO.")
        rows, texts, malformed = record(lines, args.seconds, args.settle)
    except KeyboardInterrupt:
        sys.exit("\nCancelled before recording started; nothing saved.")
    except OSError as error:
        sys.exit(f"Serial port error: {error}")
    finally:
        os.close(fd)

    if not rows:
        sys.exit("No data rows recorded; nothing saved.")

    extra = [f"# trial: {path.stem}  captured: {datetime.now().isoformat(timespec='seconds')}"]
    if args.note:
        extra.append(f"# note: {args.note}")
    with path.open("w", newline="\n") as out:
        out.write("\n".join(header[:-1] + extra + header[-1:] + texts) + "\n")

    print(f"Saved {path}")
    summarise(rows, header, malformed)


if __name__ == "__main__":
    main()
