#!/usr/bin/env python3
"""Stream the default PipeWire/PulseAudio sink monitor to PixelP over USB CDC."""

from __future__ import annotations

import argparse
import shutil
import struct
import subprocess
import sys
import time

import serial

SAMPLE_RATE = 16_000
SAMPLES_PER_PACKET = 256
PAYLOAD_BYTES = SAMPLES_PER_PACKET * 2
SYNC = b"\xA5\x5A"


def read_exact(stream, count: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < count:
        block = stream.read(count - len(chunks))
        if not block:
            raise EOFError("PipeWire capture ended unexpectedly")
        chunks.extend(block)
    return bytes(chunks)


def default_monitor_source() -> str:
    sink = subprocess.check_output(
        ["pactl", "get-default-sink"], text=True
    ).strip()
    if not sink:
        raise RuntimeError("Could not determine the default PipeWire/PulseAudio sink")
    return sink + ".monitor"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="Native USB Serial/JTAG port, e.g. /dev/ttyACM0")
    parser.add_argument("--source", help="Monitor source name; default is the current default sink monitor")
    parser.add_argument("--list-sources", action="store_true", help="List PipeWire/PulseAudio sources and exit")
    parser.add_argument("--rate", type=int, default=SAMPLE_RATE, help="Capture sample rate (firmware expects 16000)")
    args = parser.parse_args()

    if args.list_sources:
        if not shutil.which("pactl"):
            raise RuntimeError("pactl was not found; install pulseaudio-utils")
        subprocess.run(["pactl", "list", "short", "sources"], check=True)
        return 0

    if not args.port:
        parser.error("a serial port is required unless --list-sources is used")
    if args.rate != SAMPLE_RATE:
        parser.error("This firmware build expects --rate 16000")
    if not shutil.which("parec") or not shutil.which("pactl"):
        raise RuntimeError("Install pulseaudio-utils (provides pactl and parec) for PipeWire-Pulse capture")

    source = args.source or default_monitor_source()
    command = [
        "parec",
        "--device", source,
        "--format=s16le",
        f"--rate={SAMPLE_RATE}",
        "--channels=1",
        "--raw",
    ]

    print(f"Capturing {source} at {SAMPLE_RATE} Hz, mono, signed 16-bit PCM", file=sys.stderr)
    print(f"Sending packets to {args.port}. Stop with Ctrl+C.", file=sys.stderr)

    capture = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=None, bufsize=0)
    sequence = 0
    try:
        # Native ESP32-S3 USB Serial/JTAG is CDC-like; baud is ignored by the USB link.
        port = serial.Serial(port=None, baudrate=921600, timeout=1, write_timeout=2,
                             dsrdtr=False, rtscts=False)
        port.port = args.port
        port.dtr = False
        port.rts = False
        with port:
            time.sleep(0.15)
            while True:
                pcm = read_exact(capture.stdout, PAYLOAD_BYTES)
                packet = SYNC + struct.pack("<H", sequence) + pcm
                port.write(packet)
                sequence = (sequence + 1) & 0xFFFF
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
    finally:
        capture.terminate()
        try:
            capture.wait(timeout=1)
        except subprocess.TimeoutExpired:
            capture.kill()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError, serial.SerialException, EOFError) as exc:
        print(f"pixelp_pipewire: {exc}", file=sys.stderr)
        raise SystemExit(1)
