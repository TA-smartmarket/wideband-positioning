"""Read-only sniff of the three UWB boards + server state check.

Does not write anything to the serial ports (safe while monitoring).
"""
import json
import sys
import threading
import time
import urllib.request

import serial

PORTS = ["COM9", "COM10", "COM11"]
SECONDS = 8
buffers = {p: [] for p in PORTS}
errors = {}


def reader(port):
    try:
        s = serial.Serial(port, 115200, timeout=0.3)
        t0 = time.time()
        while time.time() - t0 < SECONDS:
            data = s.read(4096)
            if data:
                buffers[port].append(data.decode("utf-8", "replace"))
        s.close()
    except Exception as e:
        errors[port] = str(e)


threads = [threading.Thread(target=reader, args=(p,)) for p in PORTS]
for t in threads:
    t.start()
for t in threads:
    t.join()

for p in PORTS:
    print("=" * 70)
    print(f"### {p}")
    if p in errors:
        print(f"  !! cannot open: {errors[p]}")
        continue
    text = "".join(buffers[p]).strip()
    if not text:
        print("  (no output in the last 8 s)")
    else:
        lines = [ln for ln in text.splitlines() if ln.strip()]
        for ln in lines[-18:]:
            print("  " + ln)

print("=" * 70)
print("### server /api/v1/state")
try:
    with urllib.request.urlopen("http://127.0.0.1:8080/api/v1/state", timeout=5) as r:
        st = json.load(r)
    print("  anchors:", len(st.get("anchors", [])))
    for a in st.get("anchors", []):
        print(f"    {a['id']:<10} pos=({a['x']},{a['y']}) online={a['online']}")
    print("  tags   :", len(st.get("tags", [])))
    for t in st.get("tags", []):
        print(f"    {t['id']:<10} ({t['x']:.2f},{t['y']:.2f}) online={t['online']}")
    print("  links  :", len(st.get("links", [])))
    for l in st.get("links", [])[:6]:
        print(f"    {l['src']} -> {l['dst']}  {l['range']:.3f} m")
except Exception as e:
    print("  !! state fetch failed:", e)

print("### server /api/v1/devices")
try:
    with urllib.request.urlopen("http://127.0.0.1:8080/api/v1/devices", timeout=5) as r:
        dv = json.load(r)
    for d in dv.get("devices", []):
        print(f"    {d.get('device_id'):<10} role={d.get('role')} id={d.get('id')} online={d.get('online')}")
except Exception as e:
    print("  !! devices fetch failed:", e)
