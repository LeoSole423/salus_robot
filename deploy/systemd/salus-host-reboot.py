"""Handle one request from the local maintenance socket."""

import json
from pathlib import Path
import subprocess
import sys


def handle_request(request, boot_id, run_reboot):
    if request == b"status\n":
        return {"state": "ready", "boot_id": boot_id}
    if request != f"reboot {boot_id}\n".encode("ascii"):
        return {"state": "rejected", "boot_id": boot_id}
    if run_reboot():
        return {"state": "requested", "boot_id": boot_id}
    return {"state": "failed", "boot_id": boot_id}


def queue_reboot():
    try:
        result = subprocess.run(
            ["/usr/bin/systemctl", "--no-block", "--no-ask-password", "reboot"],
            stdin=subprocess.DEVNULL,
            stdout=sys.stderr,
            timeout=4,
            check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired) as error:
        print(error, file=sys.stderr)
        return False


def main():
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    request = sys.stdin.buffer.readline(64)
    response = handle_request(request, boot_id, queue_reboot)
    print(json.dumps(response), flush=True)


if __name__ == "__main__":
    main()
