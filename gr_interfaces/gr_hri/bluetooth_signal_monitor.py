#/usr/bin/python
import bluetooth
import time

class BluetoothMonitor(object):
    def __init__(self):
        self.uuid = "00001200-0000-1000-8000-00805f9b34fb" # PnP ZOTAC-PC
        self.text = "quit"

    def setup(self):
        port = None
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
        if port is not None:
            print "waiting for connection on RFCOMM channel", port
            return True
        else:
            print "turn bluetooth on"
            return False

    def connect_with_client(self):
        self.client_sock, self.client_info = self.sock.accept()
        print "CLIENT INFO ", self.client_info
        try:
            self.client_sock.send("Felicidades estas conectado")
        except IOError:
            print "error"
            pass


    def send_to_client(self, text):
        print "SEND"
        self.client_sock.send(text);

    def receive_from_client(self):
        print "Received"
        while True:
            data = self.client_sock.recv(1024)
            print "received %s" % data
            self.send_to_client("recibido:" + data)

    def close(self):
        self.client_sock.close()
        self.sock.close()

if __name__ == '__main__':
    bluetooth_monitor = BluetoothMonitor()
    if bluetooth_monitor.setup():
        bluetooth_monitor.connect_with_client()
        bluetooth_monitor.receive_from_client()
        bluetooth_monitor.close()
