# https://docs.python.org/3/library/telnetlib.html
from telnetlib import Telnet
import time
from uav_launcher.catapult import Catapult


record_time = time.strftime('%Y%m%d-%H%M%S')
f = open(f"logs/log_{record_time}.log", "w+")

store = []

catapult = Catapult()

tn = Telnet(f'192.168.137.{input("IP ending:")}', 23)
try:
    while True:
        newline:str = tn.read_until(match=b'\n', timeout=1).decode("utf-8")
        print(newline)
        if newline == "":
            raise
        if newline.find("Launch")>0:
            catapult.launch(7, 150)
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

input("press enter to reset catapult")

catapult.set_location(0)
