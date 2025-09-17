#!/usr/bin/env python3
import json
import os
import re
import sys
from pathlib import Path

def load_json(path: Path):
    if not path.is_file():
        print(f"ERROR: JSON file not found: {path}", file=sys.stderr)
        sys.exit(1)
    return json.loads(path.read_text())

def add_or_replace(export_file: Path, var: str, val: str):
    lines = export_file.read_text().splitlines() if export_file.exists() else []
    pattern = re.compile(rf"^export\s+{re.escape(var)}=")
    out, found = [], False

    for line in lines:
        if pattern.match(line):
            out.append(f"export {var}={val}")
            found = True
        else:
            out.append(line)

    if not found:
        out.append(f"export {var}={val}")

    export_file.write_text("\n".join(out).rstrip() + "\n")

def main():
    print("Setting up user environment variables...")
    print("Available FC_TYPE options: PX4_SIMULATED, PX4, ARDUPILOT, ANAFI")
    print("Available OBC_TYPE options: PI, JETSON, DESKTOP")
    print("Available CAM_TYPE options: RPI5, ZT30, SIM_UE, SIM_IS, NONE")

    home = Path.home()
    JSON_FILE    = home / "auspex_params" / "platform_properties" / "platform_properties.json"
    AUSPEX_HOME  = home / "AUSPEX"
    EXPORT_FILE  = AUSPEX_HOME / "utils" / "user_exports.sh"

    data = load_json(JSON_FILE)

    fc_type  = data["platform_details"]["flight_controller"]
    obc_type = data["platform_details"]["offboard_controller"]

    cams = [
        s["model"]
        for s in data.get("sensors", [])
        if "camera" in s.get("type", "").lower()
    ]

    cam_type = f"\"{';'.join(cams)}\""

    EXPORT_FILE.parent.mkdir(parents=True, exist_ok=True)

    add_or_replace(EXPORT_FILE, "FC_TYPE",  fc_type)
    add_or_replace(EXPORT_FILE, "OBC_TYPE", obc_type)
    add_or_replace(EXPORT_FILE, "CAM_TYPE", cam_type)

    print(f"Updated {EXPORT_FILE} with:")
    print(f"   FC_TYPE  = {fc_type}")
    print(f"   CAM_TYPE = {cam_type}")
    print(f"   OBC_TYPE = {obc_type}")

if __name__ == "__main__":
    main()
