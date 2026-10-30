"""A diagnostic SSH ProxyCommand with a connection-local TCP MSS.

Does not change host MTU, routes, SSH authentication or host-key verification.
Use only for an explicitly selected measurement host, for example:
  ssh -o 'ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p' HOST
"""
from __future__ import annotations

import argparse
import os
import selectors
import socket
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument("--mss", type=int, default=1024)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or not 512 <= args.mss <= 1200:
        parser.error("expected port 1..65535 and diagnostic MSS 512..1200")
    addresses = socket.getaddrinfo(args.host, args.port, type=socket.SOCK_STREAM)
    family, kind, protocol, _, address = addresses[0]
    with socket.socket(family, kind, protocol) as stream:
        stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_MAXSEG, args.mss)
        stream.settimeout(10.)
        stream.connect(address)
        stream.settimeout(None)
        with selectors.DefaultSelector() as poll:
            poll.register(sys.stdin.buffer, selectors.EVENT_READ, "stdin")
            poll.register(stream, selectors.EVENT_READ, "remote")
            while True:
                for key, _ in poll.select():
                    if key.data == "stdin":
                        chunk = os.read(sys.stdin.fileno(), 65536)
                        if chunk:
                            stream.sendall(chunk)
                        else:
                            poll.unregister(sys.stdin.buffer)
                            stream.shutdown(socket.SHUT_WR)
                    else:
                        chunk = stream.recv(65536)
                        if not chunk:
                            return
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()


if __name__ == "__main__":
    main()
