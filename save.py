import UDPComms
import time
import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()


odom  = UDPComms.Subscriber(8810, timeout=0)
lidar = UDPComms.Subscriber(8110, timeout=0)

channels = [odom, lidar]


def get_event(f,sub):
    try:
        msg = sub.recv()
    except:
        return None
    out = json.dumps([time.monotonic() - start_time, sub.port, msg])
    f.write(out + "\n")


start_time = time.monotonic()
with open(args.filename, "w+") as f:
    while 1:
        for chan in channels:
            get_event(f,chan)

