# https://docs.python.org/3/library/telnetlib.html
from telnetlib import Telnet
import time
#from uav_launcher.catapult import Catapult

CATAPULT = False


record_time = time.strftime('%Y%m%d-%H%M%S')
f = open(f"logs/log_{record_time}.log", "w+")

store = []

if CATAPULT: catapult = Catapult()

tn = Telnet(f'192.168.137.{input("IP ending:")}', 23)
try:
    while True:
        newline:str = tn.read_until(match=b'\n', timeout=1).decode("utf-8")
        print(newline)
        if newline == "":
            pass
        if newline.find("Launch")>0:
            if CATAPULT: catapult.launch(8, 150)
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
        if line == "(delayy)(C0) waiting":
            continue
        filtered = line.replace("\r\n", "").split("(C0)")[1]
        filtered = filtered.split("\t")
        newstore.append(filtered)
    except:
        print("failed to parse a line")

import pandas as pd
pd.DataFrame(newstore).to_csv(f"logs/{record_time}_processed.csv")

if CATAPULT: input("press enter to reset catapult")

if CATAPULT: catapult.set_location(0)
