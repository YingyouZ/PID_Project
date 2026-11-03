# -*- coding: utf-8 -*-
"""Sniff FPGA F21 UART traffic with a USB-TTL adapter.

Wiring:
    USB-TTL RX  -> FPGA F21 (FPGA TX)
    USB-TTL GND -> FPGA GND
    USB-TTL TX  -> leave unconnected
    USB-TTL VCC -> leave unconnected (FPGA already powered)

Usage:
    python sniff_fpga_uart.py COM7
"""
import sys
import time

import serial


def sniff(port: str, baud: int, seconds: float = 1.5) -> bytes:
    s = serial.Serial(port, baud, timeout=0.2)
    s.reset_input_buffer()
    end = time.time() + seconds
    data = bytearray()
    while time.time() < end:
        chunk = s.read(4096)
        if chunk:
            data.extend(chunk)
    s.close()
    return bytes(data)


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    for baud in (1_000_000, 115_200, 9_600):
        try:
            data = sniff(port, baud)
        except Exception as exc:
            print(f"{baud:>9}: ERROR {exc}")
            continue

        print(f"\n{baud:>9}: {len(data)} bytes")
        if not data:
            print("  (no data; check GND, F21, FPGA bitstream)")
            continue

        preview = data[:128]
        print("  first bytes:", preview.hex(" "))
        pos = data.find(b"\xaa\x55")
        if pos >= 0:
            print(f"  FOUND AA55 at offset {pos}")
            print("  frame head:", data[pos:pos + 32].hex(" "))
            print("  => FPGA UART format is present")
            return 0
        print("  no AA55 header in this capture")
        print("  => wrong baud/bitstream, or F21 is not the link TX pin")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
