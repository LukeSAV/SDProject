from bluetooth import *
import sys


if __name__ == "__main__":

    # Discovers nearby devices and prints their names and addresses
    nearby_devices = discover_devices(lookup_names=True)
    print("found %d devices" % len(nearby_devices))

    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))



class BluetoothManager:
    """Establish and save a connection to the Arduino"""
    def __init__(self, addr):
        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        service_matches = find_service( uuid = uuid, address = addr )

        if len(service_matches) == 0:
            print("couldn't find the SampleServer service =(")
            sys.exit(0)

        first_match = service_matches[0]
        port = first_match["port"]
        name = first_match["name"]
        host = first_match["host"]

        print("connecting to \"%s\" on %s" % (name, host))

        self.sock = BluetoothSocket(RFCOMM)
        self.sock.connect((host, port))

    def close(self):
        self.sock.close()
        print("Closing Bluetooth Manager")