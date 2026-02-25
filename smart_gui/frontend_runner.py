"""Helper process used by roslaunch to start the Flutter frontend command."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


def _default_frontend_dir() -> str:
    custom = os.environ.get("SMART_GUI_FRONTEND_DIR")
    if custom:
        return os.path.expanduser(custom)

    module_path = Path(__file__).resolve()
    package_root = module_path.parents[1]
    candidates = [package_root / "frontend"]
    for ancestor in module_path.parents:
        candidates.append(ancestor / "src" / "smart_gui" / "frontend")
    candidates.extend(Path.home().glob("*/src/smart_gui/frontend"))

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(package_root / "frontend")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Smart GUI frontend command")
    parser.add_argument("--frontend-dir", default="")
    parser.add_argument("--frontend-command", required=True)
    args, _ = parser.parse_known_args()

    workdir = (
        os.path.expanduser(args.frontend_dir)
        if args.frontend_dir
        else _default_frontend_dir()
    )
    cmd = args.frontend_command
    proc = subprocess.Popen(cmd, cwd=workdir, shell=True)
    try:
        return proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        return 130


if __name__ == "__main__":
    sys.exit(main())
