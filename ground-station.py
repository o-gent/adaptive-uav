# https://docs.python.org/3/library/telnetlib.html
from telnetlib import Telnet
import time

record_time = time.strftime('%Y%m%d-%H%M%S')
f = open(f"logs/log_{record_time}.log", "w+")

store = []

tn = Telnet('192.168.137.103', 23)
try:
    while True:
        newline = tn.read_until(match=b'\n', timeout=1).decode("utf-8")
        print(newline)
        if newline == "":
            raise
        store.append(newline)
except:
    print("Connection closed")
    tn.close()
    f.writelines(store)
    f.close()

log = store[37:]
newstore = []
for line in log:
    try:
        filtered = line.replace("\r\n", "").split("(C1)")[1].split("    ")
        newstore.append(filtered)
    except:
        print("failed to parse a line")

import pandas as pd
pd.DataFrame(newstore).to_csv(f"logs/{record_time}_processed.csv")
