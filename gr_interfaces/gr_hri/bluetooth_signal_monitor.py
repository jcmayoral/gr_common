#/usr/bin/python
import bluetooth
import time

class BluetoothMonitor(object):
    def __init__(self):
        self.uuid = "00001200-0000-1000-8000-00805f9b34fb" # PnP ZOTAC-PC
        self.setup()
        #self.show_devices()

    def setup(self):
        #self.nearby_devices = bluetooth.discover_devices(lookup_names=True)
        #print self.nearby_devices
        addr = "A8:5E:45:02:83:23"
        self.service_matches = bluetooth.find_service(address=addr)
        for i,service in enumerate(self.service_matches):
            #print i, service.keys()
            if service["protocol"] == "RFCOMM":
                print i, service
                port = service["port"]
                host = service["host"]
                name = service["name"]
                #break

        #first_match = self.service_matches[0]
        #port = first_match["port"]
        #name = first_match["name"]
        #host = first_match["host"]

        #wprint (name, host)
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        #server_sock.bind(("", bluetooth.PORT_ANY))
        print sock.getsockname()
        #server_sock.listen(1)
        sock.connect((host,port))

        #bluetooth.advertise_service(server_sock, "SampleServer", service_id=self.uuid,
        #                            service_classes=[self.uuid, bluetooth.SERIAL_PORT_CLASS],
        #                                profiles=[bluetooth.SERIAL_PORT_PROFILE],)
        print "waiting for connection on RFCOMM channel", port
        time.sleep(3)
        print "SEND TEXT"
        while True:
            text = raw_input()
            print text
            if text == "quit":
                break
            sock.send(text);
        sock.close()


    def show_devices(self):
        print "Found {} devices".format(len(self.nearby_devices))
        for addr, name in self.nearby_devices:
            print "{} {}".format(addr, name)


if __name__ == '__main__':
    bluetooth_monitor = BluetoothMonitor()
