#!/usr/bin/env python3
import os
import re
import sys
from pathlib import Path
import yaml

def load_yaml(path: Path):
    if not path.is_file():
        print(f"ERROR: YAML file not found: {path}", file=sys.stderr)
        sys.exit(1)
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)

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
    print("Available OBC_TYPE options: PI, JETSON_XAVIER_NX, JETSON_ORIN_NX, DESKTOP")
    print("Available CAM_TYPE options: RPI5, ZT30, SIM_UE, SIM_IS, NONE")

    home = Path.home()
    AUSPEX_HOME  = home / "AUSPEX"
    YAML_FILE    = AUSPEX_HOME / "params" / "platform_properties" / "platform_properties.yaml"
    EXPORT_FILE  = AUSPEX_HOME / "utils" / "user_exports.sh"

    data = load_yaml(YAML_FILE)
    general = data.get("general", {})

    platform_name = f'"{general["platform_id"]}"'
    fc_type  = general["flight_controller"]
    obc_type = general["offboard_controller"]

    cams = [
        s["model"]
        for s in data.get("peripheral_devices", [])
        if "camera" in s.get("type", "").lower()
    ]

    cam_type = "\"NONE\"" if not cams else f"\"{';'.join(cams)}\""

    EXPORT_FILE.parent.mkdir(parents=True, exist_ok=True)

    add_or_replace(EXPORT_FILE, "PLATFORM_NAME", platform_name)
    add_or_replace(EXPORT_FILE, "FC_TYPE",  fc_type)
    add_or_replace(EXPORT_FILE, "OBC_TYPE", obc_type)
    add_or_replace(EXPORT_FILE, "CAM_TYPE", cam_type)

    print(f"Updated {EXPORT_FILE} with:")
    print(f"   PLATFORM_NAME = {platform_name}")
    print(f"   FC_TYPE  = {fc_type}")
    print(f"   CAM_TYPE = {cam_type}")
    print(f"   OBC_TYPE = {obc_type}")

if __name__ == "__main__":
    main()
