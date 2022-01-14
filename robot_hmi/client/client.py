#! /usr/bin/env python3

import cmd
import socket
import threading

class Client(threading.Thread, cmd.Cmd):

    prompt = ""

    def __init__(self, connection):
        threading.Thread.__init__(self)
        cmd.Cmd.__init__(self)

        self.kill = threading.Event()

        self.connection = connection
        self.reader = connection.makefile("rb", -1)
        self.writer = connection.makefile("wb", 0)

    def start(self):
        super().start()
        
        self.cmdloop()

        self.kill.set()
        self.cleanup()

    def cleanup(self):
        print("Cleanup")
        self.writer.flush()
        self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()

    def run(self):
        try:
            while not self.kill.is_set():
                self.handle_server_response()
        except (BrokenPipeError, ConnectionResetError) as err:
            print(err)

    def handle_server_response(self):
        data = self.reader.readline()
        if not data: return
        print(f"Received: {data.decode('utf-8')}")
        # raise KeyboardInterrupt

    # - Client commands - #
    def do_ping(self, arg):
        """Ping the server"""
        self.writer.write(b"ping\n")
        self.writer.flush()

    def do_quit(self, arg):
        """Quit cmd"""
        return True



if __name__ == "__main__":
    client = Client(socket.create_connection(("192.168.1.74", 8888)))

    client.start()

