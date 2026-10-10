"""Build-only entry point for the pinned, project-local ESP-IDF installation.

No arbitrary idf.py actions, serial ports, flashing, or monitoring are accepted.
The SDK installation itself is explicit; this runner never downloads software.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ROLES = ("container", "pocketqube", "ground_radio")


def git(sdk, *args):
    return subprocess.run(["git", "-C", str(sdk), *args], check=True,
                          capture_output=True, text=True).stdout.rstrip("\n")


def verify_installation(root=ROOT):
    """Validate local SDK provenance; return lock and paths, or raise without installing anything."""
    base = root / "software/esp_idf"
    lock = json.loads((base / "sdk.lock.json").read_text(encoding="utf-8"))
    sdk = base / lock["sdk_directory"]
    tools = base / lock["tools_directory"]
    python_env = tools / lock["python_environment"]
    if not (sdk / "tools/idf.py").is_file() or not (python_env / "bin/python").is_file():
        raise ValueError("Pinned SDK/Python environment missing; follow the architecture setup instructions.")
    if git(sdk, "rev-parse", "HEAD") != lock["commit"]:
        raise ValueError("SDK commit differs from sdk.lock.json")
    if git(sdk, "remote", "get-url", "origin") != lock["repository"]:
        raise ValueError("SDK origin differs from sdk.lock.json")
    if git(sdk, "status", "--porcelain", "--untracked-files=no"):
        raise ValueError("SDK or submodule tracked files have local modifications")
    submodules = git(sdk, "submodule", "status", "--recursive")
    if not submodules or any(not line.startswith(" ") for line in submodules.splitlines()):
        raise ValueError("SDK submodules are missing, conflicted, or not at their pinned commits")
    digest = hashlib.sha256((sdk / "tools/tools.json").read_bytes()).hexdigest()
    if digest != lock["tools_manifest_sha256"]:
        raise ValueError("SDK tool manifest differs from sdk.lock.json")
    return lock, sdk, tools, python_env


def build_command(root, role):
    """Construct the fixed build-only argv; caller passes SDK variables in a child environment."""
    base = root / "software/esp_idf"
    build = base / ".local/build-ninja" / role
    # Pass every path as an argument, never interpolate it into shell code.
    return ["bash", "-c",
            'set -e; . "$IDF_PATH/export.sh" >&2; '
            'exec "$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" "$@"',
            "cansat-build-only", "-G", "Ninja", "-C", str(base / "apps" / role), "-B", str(build),
            "-D", "IDF_TARGET=esp32", "-D", f"SDKCONFIG={build / 'sdkconfig'}", "build"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "build"))
    parser.add_argument("role", nargs="?", choices=(*ROLES, "all"), default="all")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args(argv)
    if not 1 <= args.jobs <= 16:
        parser.error("--jobs must be between 1 and 16")
    try:
        lock, sdk, tools, python_env = verify_installation()
        print(f"SDK_PIN_OK: {lock['tag']} commit={lock['commit']} target={lock['idf_target']}", flush=True)
        if args.action == "check":
            return 0
        env = os.environ.copy()
        env.update({"IDF_PATH": str(sdk), "IDF_TOOLS_PATH": str(tools),
                    "IDF_PYTHON_ENV_PATH": str(python_env), "IDF_TARGET": "esp32",
                    "IDF_COMPONENT_MANAGER": "0", "IDF_CCACHE_ENABLE": "0",
                    "IDF_PY_BUILD_JOBS": str(args.jobs), "PYTHONNOUSERSITE": "1",
                    "PYTHONDONTWRITEBYTECODE": "1",
                    # SDK export prepends its installed compiler/CMake/Ninja.
                    # Do not inherit another SDK or Homebrew build-tool paths.
                    "PATH": str(python_env / "bin") + ":/usr/bin:/bin:/usr/sbin:/sbin"})
        for role in ROLES if args.role == "all" else (args.role,):
            print(f"BUILD_ONLY: {role}; no serial device or flash action", flush=True)
            subprocess.run(build_command(ROOT, role), env=env, check=True)
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"ESP_IDF_BUILD_FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
