"""Forward one maintenance request from an SSH forced command."""

import os
import re
import socket
import sys


SOCKET_PATH = "/run/salus-maintenance/reboot.sock"
REQUEST = re.compile(rb"(?:status|reboot [0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12})\n")


def relay(request, socket_path=SOCKET_PATH):
    if not REQUEST.fullmatch(request):
        raise ValueError("Invalid maintenance request")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(7)
        client.connect(socket_path)
        client.sendall(request)
        with client.makefile("rb") as response:
            line = response.readline(513)
    if len(line) > 512 or not line.endswith(b"\n"):
        raise ValueError("Invalid maintenance response")
    return line


def main():
    try:
        if os.environ.get("SSH_ORIGINAL_COMMAND") != "maintenance":
            raise ValueError("Invalid SSH maintenance command")
        response = relay(sys.stdin.buffer.readline(64))
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    sys.stdout.buffer.write(response)
    return 0


if __name__ == "__main__":
    sys.exit(main())
