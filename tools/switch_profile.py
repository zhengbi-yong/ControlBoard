#!/usr/bin/env python3
"""Utility to switch the CONTROL_BOARD_PROFILE macro.

The script rewrites ``User/APP/control_board_profile.h`` so that the
``CONTROL_BOARD_PROFILE`` macro matches the requested mechanical layout.  The
firmware build then targets the correct subset of actuators without needing to
manually edit any project files.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PROFILE_HEADER = REPO_ROOT / "User" / "APP" / "control_board_profile.h"

PROFILE_ALIASES = {
    "neck": "CONTROL_BOARD_PROFILE_NECK",
    "left_arm": "CONTROL_BOARD_PROFILE_LEFT_ARM",
    "right_arm": "CONTROL_BOARD_PROFILE_RIGHT_ARM",
    "left_leg": "CONTROL_BOARD_PROFILE_LEFT_LEG",
    "right_leg": "CONTROL_BOARD_PROFILE_RIGHT_LEG",
    "waist": "CONTROL_BOARD_PROFILE_WAIST",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rewrite User/APP/control_board_profile.h so that the firmware is "
            "built for the requested control-board layout."
        )
    )
    parser.add_argument(
        "profile",
        choices=sorted(PROFILE_ALIASES.keys()),
        help=(
            "Target control-board layout.  The value maps to the "
            "CONTROL_BOARD_PROFILE_* macros defined in control_board_profile.h."
        ),
    )
    return parser.parse_args()


def update_profile_macro(target_macro: str) -> bool:
    """Update the CONTROL_BOARD_PROFILE definition.

    Parameters
    ----------
    target_macro:
        Name of the CONTROL_BOARD_PROFILE_* macro that should be activated.

    Returns
    -------
    bool
        True when the macro was successfully updated, False otherwise.
    """

    if not PROFILE_HEADER.exists():
        raise FileNotFoundError(f"Cannot locate {PROFILE_HEADER}")

    original = PROFILE_HEADER.read_text(encoding="utf-8")
    pattern = re.compile(r"(^#define\\s+CONTROL_BOARD_PROFILE\\s+)(CONTROL_BOARD_PROFILE_[A-Z_]+)", re.MULTILINE)

    def _replacement(match: re.Match[str]) -> str:
        prefix, _ = match.groups()
        return f"{prefix}{target_macro}"

    updated, count = pattern.subn(_replacement, original, count=1)

    if count == 0:
        return False

    if updated != original:
        PROFILE_HEADER.write_text(updated, encoding="utf-8")

    return True


def main() -> int:
    args = parse_args()
    target_macro = PROFILE_ALIASES[args.profile]

    try:
        changed = update_profile_macro(target_macro)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1

    if not changed:
        print(
            "Failed to update CONTROL_BOARD_PROFILE. Ensure the macro is present in the header.",
            file=sys.stderr,
        )
        return 2

    print(f"CONTROL_BOARD_PROFILE set to {target_macro} in {PROFILE_HEADER.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
