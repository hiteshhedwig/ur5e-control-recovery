#!/usr/bin/python3
"""Extract pinned upstream Noetic binaries locally; never changes system packages."""
import hashlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = {
    "ur-gazebo": ("1.5.0-1focal.20250521.011251", "c11fddddfcc5d64eca7aad12269b0619166cbc5d72d29c43462ed75600e44d5e"),
    "ur-description": ("1.5.0-1focal.20250520.012707", "8e8eda854cc23d8d344590e3433ebe49a012cb04b711c4685d0aee4cf64f2a28"),
    "joint-trajectory-controller": ("0.22.0-1focal.20250521.005320", "cd252e7ba5b5c22833878ff70d92cc1c2922b8b5da158508cbb2fc4837e021f2"),
    "effort-controllers": ("0.22.0-1focal.20250521.005323", "15f4939472de4076fdeda491d913dbc94d3866b83ddabc4c434fbe2f9288baa9"),
}

def main():
    archives = ROOT / ".deps/debs"
    prefix = ROOT / ".deps/root"
    archives.mkdir(parents=True, exist_ok=True)
    prefix.mkdir(parents=True, exist_ok=True)
    for name, (version, digest) in PACKAGES.items():
        package = "ros-noetic-" + name
        filename = "{}_{}_amd64.deb".format(package, version)
        archive = archives / filename
        if not archive.exists():
            url = "http://packages.ros.org/ros/ubuntu/pool/main/r/{}/{}".format(package, filename)
            temporary = archive.with_suffix(".part")
            subprocess.run(["curl", "--fail", "--location", "--silent", "--show-error", "--max-time", "120", url, "-o", str(temporary)], check=True)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != digest:
                raise RuntimeError("Checksum mismatch: " + filename)
            temporary.replace(archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != digest:
            raise RuntimeError("Checksum mismatch: " + filename)
        subprocess.run(["dpkg-deb", "--extract", str(archive), str(prefix)], check=True)
        print("Verified and extracted", filename, flush=True)

if __name__ == "__main__":
    main()
