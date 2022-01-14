#! /usr/bin/env python3
import socketserver


class CustomServer(socketserver.ThreadingTCPServer):
    def __init__(self, server_address, request_handler_class):
        super().__init__(server_address, request_handler_class, True)


class CustomHandler(socketserver.StreamRequestHandler):
    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server)

        self.commands = dict()

    def setup(self):
        super().setup()
        print(f">>> {self.client_address[0]}")

    def handle(self):
        try:
            while True:
                data = self.rfile.readline()
                if not data: break
                data = data.decode('utf-8')

                if "ping" in data:
                    self.cmd_ping()
        except (ConnectionResetError, EOFError):
            pass
        except KeyboardInterrupt:
            return

    def finish(self):
        print(f"<<< {self.client_address[0]}")
        super().finish()

    # - Server commands - #
    def cmd_ping(self):
        self.wfile.write(b"pong\n")
        self.wfile.flush()


def main():
    server_address = ("0.0.0.0", 8888)

    server = CustomServer(server_address, CustomHandler)

    print("Server open")

    server.serve_forever()


if __name__ == "__main__":
    main()

