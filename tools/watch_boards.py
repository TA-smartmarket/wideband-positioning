"""Timestamped serial sniff of the three boards — measures the
'device lost' / 'device added' cadence that makes the OLED flicker.

    python tools/watch_boards.py [seconds]
"""
import sys
import threading
import time

import serial

PORTS = ["COM9", "COM10", "COM11"]
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 12
lines = {p: [] for p in PORTS}
errors = {}


def reader(port):
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0.2
        s.dtr = False          # do not reset the ESP32 when opening
        s.rts = False
        s.open()
        s.dtr = False
        s.rts = False
        t0 = time.time()
        buf = ""
        while time.time() - t0 < SECONDS:
            data = s.read(4096)
            if not data:
                continue
            buf += data.decode("utf-8", "replace")
            while "\n" in buf:
                ln, buf = buf.split("\n", 1)
                ln = ln.strip()
                if ln:
                    lines[port].append((time.time() - t0, ln))
        s.close()
    except Exception as e:
        errors[port] = str(e)


threads = [threading.Thread(target=reader, args=(p,)) for p in PORTS]
for t in threads:
    t.start()
for t in threads:
    t.join()

INTERESTING = ("device lost", "device added", "blinked", "range ", "wifi", "mqtt", "cfg", "ap]")

for p in PORTS:
    print("=" * 72)
    print(f"### {p}   ({SECONDS:.0f} s window)")
    if p in errors:
        print("   !!", errors[p])
        continue
    ev = [(t, l) for t, l in lines[p] if any(k in l for k in INTERESTING)]
    counts = {}
    for _, l in ev:
        key = l.split()[0] + " " + (l.split()[1] if len(l.split()) > 1 else "")
        key = key.replace("range", "range")
        counts[key] = counts.get(key, 0) + 1
    print("   event counts:")
    for k, v in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"     {v:4d}  {k}")
    print("   last 12 events:")
    for t, l in ev[-12:]:
        print(f"     [{t:6.2f}s] {l[:78]}")
