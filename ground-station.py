"""
co-ordinate catapult and UAV
"""

import websocket
import pandas as pd
import time

ws = websocket.WebSocket()

ws.connect(f"ws://192.168.137.{input('ip end number')}/ws")

# start catapult

# create datastore
df = pd.DataFrame(columns=['x', 'y', 'z'])

ws.send("start")
print(ws.recv())

# start UAV
while True:
    try:
        s = time.time()
        ws.send("d")
        print(ws.recv())
        #print(ws.recv())
        #print(ws.recv())
        print(time.time() - s)
    except:
        break