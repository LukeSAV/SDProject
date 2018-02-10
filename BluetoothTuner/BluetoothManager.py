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
    def __init__(self, address):
        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        print(address)

        self.sock = BluetoothSocket(RFCOMM)
        self.sock.connect((address,1 ))
    def send(self,data):
        self.sock.send(data)

    def close(self):
        self.sock.close()
        print("Closing Bluetooth Manager")