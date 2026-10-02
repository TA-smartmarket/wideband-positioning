"""Print only the config lines ('show' output) for each board."""
import re
import sys
import threading
import time

import serial

PORTS = sys.argv[1:] or ["COM9", "COM10", "COM11"]
KEYS = ("role", "id", "eui", "site", "wifi", "server", "mqtt", "base topic",
        "position", "room", "uwb mode", "filter", "rate", "wifi state")
results = {}


def query(port):
    out = []
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0.3
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(0.4)
        s.reset_input_buffer()
        s.write(b"show\n")
        s.flush()
        t0 = time.time()
        while time.time() - t0 < 4:
            data = s.read(4096)
            if data:
                out.append(data.decode("utf-8", "replace"))
        s.close()
    except Exception as e:
        out.append(f"!! {e}")
    results[port] = "".join(out)


threads = [threading.Thread(target=query, args=(p,)) for p in PORTS]
for t in threads:
    t.start()
for t in threads:
    t.join()

for p in PORTS:
    print("=" * 60)
    print(f"### {p}")
    for ln in results.get(p, "").splitlines():
        if any(ln.strip().startswith(k) for k in KEYS):
            print("  " + ln.strip())
