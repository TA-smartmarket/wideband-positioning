"""Query each board's config over the serial menu ('show').

Opening a CP210x port may pulse DTR/RTS and reset the ESP32 once; the script
sets both lines low immediately and captures the boot banner too, which is
useful in itself.

    python tools/query_boards.py            # all three ports
    python tools/query_boards.py COM9 COM11
"""
import sys
import threading
import time

import serial

PORTS = sys.argv[1:] or ["COM9", "COM10", "COM11"]
results = {}


def query(port):
    out = []
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0.3
        s.dtr = False           # keep the ESP32 out of reset
        s.rts = False
        s.open()
        s.dtr = False
        s.rts = False
        time.sleep(0.4)
        s.reset_input_buffer()
        s.write(b"show\n")
        s.flush()
        t0 = time.time()
        while time.time() - t0 < 5:
            data = s.read(4096)
            if data:
                out.append(data.decode("utf-8", "replace"))
        s.close()
    except Exception as e:
        out.append(f"!! cannot open: {e}")
    results[port] = "".join(out)


threads = [threading.Thread(target=query, args=(p,)) for p in PORTS]
for t in threads:
    t.start()
for t in threads:
    t.join()

for p in PORTS:
    print("=" * 68)
    print(f"### {p}")
    text = results.get(p, "")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    for ln in lines[-30:]:
        print("  " + ln)
    if not lines:
        print("  (no output)")
