"""Read one evidence archive through the connection manager's pinned SSH adapter.

Executed inside the explicitly selected installed manager, via remote.fetch.
This helper has no remote command execution, upload or lifecycle operation.
"""
import json
from pathlib import PurePosixPath
import stat
import sys


def main():
    from elesim_connections.connection_manager import SshEndpoint
    from elesim_connections.secure_deployment import ParamikoConnector

    request = json.loads(sys.stdin.read())
    path = PurePosixPath(request["archive"])
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("remote archive must be an absolute, normalized path")
    endpoint = SshEndpoint.from_dict(request["endpoint"])
    with ParamikoConnector(timeout_s=10, command_timeout_s=30).connect(endpoint) as session:
        sftp = session._get_sftp()
        info = sftp.lstat(str(path))
        if not stat.S_ISREG(info.st_mode) or info.st_size > 64 * 1024**3:
            raise ValueError("remote archive must be a bounded regular file")
        with sftp.file(str(path), "rb") as stream:
            while block := stream.read(1024 * 1024):
                sys.stdout.buffer.write(block)


if __name__ == "__main__":
    main()
