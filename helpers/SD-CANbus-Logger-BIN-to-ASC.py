#!/usr/bin/env python3
"""Convert SD-CANbus-Logger SCNL binary captures to Vector-style ASC text.

Format v1: 32-byte little-endian header; 22-byte records.
Timestamps are microseconds since ESP32 boot. The converter expresses them
relative to the capture-start timestamp stored in the header. The logger does
not store wall-clock time, so the ASC date line is an explicit placeholder.
"""
import argparse
import datetime
import struct
import sys
from pathlib import Path

HEADER = struct.Struct("<4sHHHHIIQI")
RECORD = struct.Struct("<QIBB8s")


def convert(source: Path, destination: Path, channel: int = 1) -> int:
    raw = source.read_bytes()
    if len(raw) < HEADER.size:
        raise ValueError(f"File too short for header: {len(raw)} bytes")
    magic, version, header_size, record_size, _reserved, bitrate, boot_id, start_us, _reserved2 = HEADER.unpack_from(raw)
    if magic != b"SCNL":
        raise ValueError(f"Invalid magic {magic!r}; expected b'SCNL'")
    if version != 1:
        raise ValueError(f"Unsupported binary format version: {version}")
    if header_size != HEADER.size or record_size != RECORD.size:
        raise ValueError(f"Unexpected header/record sizes: {header_size}/{record_size}")
    payload = len(raw) - header_size
    complete_records, trailing = divmod(payload, record_size)
    if trailing:
        raise ValueError(f"Incomplete trailing record: {trailing} extra bytes")

    lines = [
        "date Thu Jan 01 00:00:00.000000 1970",
        "base hex  timestamps absolute",
        "// SD-CANbus-Logger ASC export",
        f"// Source: {source.name}",
        f"// Format version: {version}; bitrate: {bitrate}; boot ID: {boot_id}",
        "// Logger stores time since boot, not wall-clock time.",
        "// Timestamps below are seconds relative to capture start.",
        "// Date line is a placeholder; it is not the actual capture date.",
        "no internal events logged",
    ]

    previous_timestamp = None
    for index in range(complete_records):
        offset = header_size + index * record_size
        timestamp_us, identifier, flags, dlc, data = RECORD.unpack_from(raw, offset)
        if dlc > 8:
            raise ValueError(f"Record {index}: invalid DLC {dlc}")
        if previous_timestamp is not None and timestamp_us < previous_timestamp:
            raise ValueError(f"Record {index}: timestamp moved backwards")
        previous_timestamp = timestamp_us
        relative_us = timestamp_us - start_us
        if relative_us < 0:
            raise ValueError(f"Record {index}: timestamp precedes capture start")
        extended = bool(flags & 0x01)
        remote = bool(flags & 0x02)
        can_id = f"{identifier:X}" + ("x" if extended else "")
        timestamp = f"{relative_us / 1_000_000:.6f}"
        if remote:
            lines.append(f"{timestamp} {channel} {can_id} Rx r {dlc}")
        else:
            octets = " ".join(f"{byte:02X}" for byte in data[:dlc])
            lines.append(f"{timestamp} {channel} {can_id} Rx d {dlc}" + (f" {octets}" if octets else ""))

    destination.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")
    return complete_records


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert SD-CANbus-Logger .BIN capture to Vector-style .ASC")
    parser.add_argument("input", type=Path, help="Input CAN_XXXXX.BIN file")
    parser.add_argument("-o", "--output", type=Path, help="Output ASC file (default: same basename .ASC)")
    parser.add_argument("--channel", type=int, default=1, help="ASC channel number (default: 1)")
    args = parser.parse_args()
    if args.channel < 1:
        parser.error("--channel must be >= 1")
    output = args.output or args.input.with_suffix(".ASC")
    try:
        count = convert(args.input, output, args.channel)
    except (OSError, ValueError, struct.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"Converted {count:,} CAN records")
    print(f"Input:  {args.input} ({args.input.stat().st_size:,} bytes)")
    print(f"Output: {output} ({output.stat().st_size:,} bytes)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
