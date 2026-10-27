"""Wait for the interface selected by the role's DDS config, without DDS I/O."""
import argparse
import ipaddress
import json
from pathlib import Path
import shutil
import subprocess
import time

import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("config", type=Path)
parser.add_argument("--timeout", type=float, default=120)
args = parser.parse_args()
config = yaml.safe_load(args.config.read_text())
interface = str(config["dds"]["network_interface"]).strip()
if not interface:
    raise SystemExit("DDS network_interface must be configured")
ip_command = shutil.which("ip")
if not ip_command:
    raise SystemExit("ip command is unavailable")

deadline = time.monotonic() + max(0, args.timeout)
print(f"Waiting for DDS interface {interface} IPv4", flush=True)
while True:
    process = subprocess.run(
        [ip_command, "-j", "-4", "address", "show", "dev", interface],
        capture_output=True, text=True, timeout=5,
    )
    if process.returncode == 0:
        for device in json.loads(process.stdout):
            if "UP" not in device.get("flags", []) or device.get("operstate") not in {"UP", "UNKNOWN"}:
                continue
            for address in device.get("addr_info", []):
                if address.get("family") != "inet":
                    continue
                value = ipaddress.ip_address(address["local"])
                if not value.is_unspecified:
                    print(f"DDS interface ready: {interface} {value}", flush=True)
                    raise SystemExit(0)
    if time.monotonic() >= deadline:
        raise SystemExit(f"DDS interface {interface} has no usable IPv4 yet")
    time.sleep(min(1, max(0, deadline - time.monotonic())))
