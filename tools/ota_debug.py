"""Read the device's serial log while an OTA POST is attempted.

This is the empirical way to see why POST /update fails: the device prints the
reason (assert, Update error, key rejection) on its UART.
"""
import json
import sys
import threading
import time
import urllib.request

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
SERVER = "http://127.0.0.1:8080"
FIRMWARE = "uwb-node-1.0.3.bin"

log = []


def reader():
    try:
        s = serial.Serial()
        s.port = PORT
        s.baudrate = 115200
        s.timeout = 0.2
        s.dtr = False
        s.rts = False
        s.open()
        t0 = time.time()
        while time.time() - t0 < 40:
            d = s.read(8192)
            if d:
                log.append(d.decode("utf-8", "replace"))
        s.close()
    except Exception as e:
        log.append(f"!! serial error: {e}\n")


th = threading.Thread(target=reader)
th.start()
time.sleep(3)

# find the device + key
with urllib.request.urlopen(f"{SERVER}/api/v1/ota", timeout=5) as r:
    ota = json.load(r)
dev = next((d for d in ota["devices"] if d["ip"]), None)
print(f"target {dev['id']} at {dev['ip']} key={dev['key'][:12]}...")

url = f"http://{dev['ip']}:3232/update?key={dev['key']}"
with open(f"firmware/{FIRMWARE}", "rb") as fh:
    data = fh.read()

boundary = "----uwbtest"
body = (f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="firmware"; filename="{FIRMWARE}"\r\n'
        f"Content-Type: application/octet-stream\r\n\r\n").encode() + data + \
       f"\r\n--{boundary}--\r\n".encode()

req = urllib.request.Request(url, data=body, method="POST")
req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
req.add_header("Expect", "")
print("POSTing", len(body), "bytes ...")
try:
    with urllib.request.urlopen(req, timeout=120) as resp:
        print("response:", resp.status, resp.read()[:80])
except Exception as e:
    print("POST failed:", type(e).__name__, e)

th.join()
print("\n=== device serial during the attempt ===")
print("".join(log)[-3000:])
