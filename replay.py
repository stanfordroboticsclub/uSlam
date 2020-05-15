
import UDPComms
import time
import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

publishers = {}

start_time = time.monotonic()
with open(args.filename, "r") as f:
    for line in f:
        timestamp, port, msg = json.loads(line)

        pub = publishers.get(port, None)
        if pub == None:
            pub = UDPComms.Publisher(port)
            publishers[port] = pub

        elapsed_time = time.monotonic() - start_time
        sleep_time = timestamp - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        pub.send(msg)



