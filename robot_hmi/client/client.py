#! /usr/bin/env python3

import socket
import threading

class Client(threading.Thread):
    def __init__(self, connection):
        threading.Thread.__init__(self)
        self.connection = connection
        self.reader = connection.makefile("rb", -1)
        self.writer = connection.makefile("wb", 0)

    def start(self):
        super().start()

        self.send_ping()

        self.join()
        self.cleanup()

    def cleanup(self):
        print("Cleanup")
        self.writer.flush()
        self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()

    def run(self):
        try:
            while True:
                self.handle_server_command()
        except (BrokenPipeError, ConnectionResetError) as err:
            print(err)
        except KeyboardInterrupt:
            return

    def handle_server_command(self):
        data = self.reader.readline()
        if not data: return
        print(f"Received: {data}")
        raise KeyboardInterrupt

    def send_ping(self):
        print("Sending ping")
        self.writer.write(b"ping\n")
        self.writer.flush()


if __name__ == "__main__":
    client = Client(socket.create_connection(("192.168.1.74", 8888)))

    client.start()

