#/usr/bin/python
import bluetooth
import time

class BluetoothMonitor(object):
    def __init__(self):
        self.uuid = "00001200-0000-1000-8000-00805f9b34fb" # PnP ZOTAC-PC

    def setup(self):
        addr = "A8:5E:45:02:83:23"
        self.service_matches = bluetooth.find_service(address=addr)
        for i,service in enumerate(self.service_matches):
            if service["protocol"] == "RFCOMM":
                print i, service
                port = service["port"]
                host = service["host"]
                name = service["name"]
                #break

        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.bind(("", bluetooth.PORT_ANY))
        #print sock.getsockname()
        self.sock.listen(1)

        bluetooth.advertise_service(self.sock, "SampleServer", service_id=self.uuid,
                                    service_classes=[self.uuid, bluetooth.SERIAL_PORT_CLASS],
                                        profiles=[bluetooth.SERIAL_PORT_PROFILE],)
        print "waiting for connection on RFCOMM channel", port


    def connect_with_client(self):
        client_sock, client_info = self.sock.accept()
        print "CLIENT INFO ", client_info
        try:
            client_sock.send("Felicidades estas conectado")
        except IOError:
            print "error"
            pass

        while True:
            text = raw_input()
            print text
            if text == "quit":
                break
            client_sock.send(text);
        client_sock.close()

    def close(self):
        self.sock.close()

if __name__ == '__main__':
    bluetooth_monitor = BluetoothMonitor()
    bluetooth_monitor.setup()
    bluetooth_monitor.connect_with_client()
    bluetooth_monitor.close()
