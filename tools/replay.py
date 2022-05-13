
import UDPComms
import time
import argparse
import json

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()


# mac has a weird thing with a limmited MTU size. For long messages you need to increase it
# using methods from here. Still figuring out a nice way for this
# https://stackoverflow.com/questions/22819214/udp-message-too-long
# http://www.hackaapl.com/mazimum-transmission-unit-mtu-frame-size-in-os-x/
# sudo sysctl -w net.inet.udp.maxdgram=65535
# sudo ifconfig feth1571 mtu 9000


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



