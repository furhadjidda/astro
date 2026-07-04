#!/usr/bin/env python3
"""
save_layout.py
Dumps the current Zellij layout and merges ROS_ENABLE into all bash panes
inside real tab blocks. Handles:
  - Existing command="bash" panes
  - Bare panes with no command (converts them to bash panes)
  - Deeply nested split panes
  - Leaves swap_*, new_tab_template untouched

To add a new ROS tab, add it to ROS_TAB_CONFIG below.
"""

import subprocess
import os
import sys
import re

# ─── CONFIG ───────────────────────────────────────────────────────────────────
LAYOUT_FILE = os.path.expanduser("~/.config/zellij/layouts/project_astro.kdl")

ROS_TAB_CONFIG = {
    "ros_diag": "y",
    "ros2_apps": "y",
    # "my_new_ros_tab": "y",
}
DEFAULT_ROS_ENABLE = "n"

# Pane commands that should KEEP start_suspended and be left untouched
KEEP_SUSPENDED_CMDS = ["save_layout.py", "dump-layout", "zellij"]
# ──────────────────────────────────────────────────────────────────────────────


def dump_layout():
    result = subprocess.run(
        ["zellij", "action", "dump-layout"], capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f"Error dumping layout:\n{result.stderr}")
        sys.exit(1)
    return result.stdout


def find_block_end(lines, start):
    """Find closing brace line index for block starting at `start`."""
    depth = 0
    for i in range(start, len(lines)):
        depth += lines[i].count("{") - lines[i].count("}")
        if depth == 0:
            return i
    return len(lines) - 1


def is_special_pane(lines, start, end):
    block = "\n".join(lines[start : end + 1])
    return any(cmd in block for cmd in KEEP_SUSPENDED_CMDS)


def is_bare_pane(line):
    """A bare pane has no command= and no plugin and no children block marker."""
    if not re.search(r"\bpane\b", line):
        return False
    if "command=" in line or "plugin" in line or "borderless" in line:
        return False
    if "split_direction" in line or "children" in line:
        return False
    return True


def is_split_group(line):
    """A pane that groups children (split container) but isn't itself a terminal."""
    if not re.search(r"\bpane\b", line):
        return False
    if "command=" in line or "plugin" in line or "borderless" in line:
        return False
    if "{" in line:
        return True
    return False


def process_bash_pane(lines, start, end, ros):
    """Update or inject ROS_ENABLE and start_suspended in a bash pane block."""
    indent = re.match(r"^(\s*)", lines[start]).group(1)
    inner = indent + "    "
    special = is_special_pane(lines, start, end)

    new_block = []
    args_written = False
    suspended_written = False

    for line in lines[start : end + 1]:
        if "args" in line and "ROS_ENABLE=" in line:
            line = re.sub(r"ROS_ENABLE=[yn]", f"ROS_ENABLE={ros}", line)
            new_block.append(line)
            args_written = True
            continue
        if "start_suspended" in line:
            if special:
                new_block.append(line)
                suspended_written = True
            # else drop it — we re-add below
            continue
        new_block.append(line)

    # Inject args if missing
    if not args_written:
        for idx, line in enumerate(new_block):
            if re.search(r"\bpane\b", line) and "{" in line:
                new_block.insert(
                    idx + 1, f'{inner}args "-c" "ROS_ENABLE={ros} exec bash"'
                )
                break

    # Inject start_suspended before closing brace
    if not special and not suspended_written:
        for idx in range(len(new_block) - 1, -1, -1):
            if new_block[idx].strip() == "}":
                new_block.insert(idx, f"{inner}start_suspended true")
                break

    return new_block


def convert_bare_pane(line, ros):
    """
    Convert a bare pane line like:
        pane cwd="foo" size="50%"
    into a proper bash pane block with ROS_ENABLE and start_suspended.
    """
    indent = re.match(r"^(\s*)", line).group(1)
    inner = indent + "    "
    # Strip trailing whitespace/brace if any
    line = line.rstrip()
    # Remove trailing { if present
    line = line.rstrip("{").rstrip()
    return [
        f'{line} command="bash" {{',
        f'{inner}args "-c" "ROS_ENABLE={ros} exec bash"',
        f"{inner}start_suspended true",
        f"{indent}}}",
    ]


def process_tab_block(lines, start, end, ros):
    """Recursively process all panes within a tab block."""
    result = []
    i = start
    while i <= end:
        line = lines[i]

        # Skip plugin/borderless/children lines
        if "plugin" in line or "borderless" in line or "children" in line:
            result.append(line)
            i += 1
            continue

        # Bash pane with block
        if re.search(r"\bpane\b", line) and 'command="bash"' in line and "{" in line:
            pane_end = find_block_end(lines, i)
            result.extend(process_bash_pane(lines, i, pane_end, ros))
            i = pane_end + 1
            continue

        # Special pane (zellij, python3 save_layout, etc) — leave untouched
        if re.search(r"\bpane\b", line) and "command=" in line and "{" in line:
            pane_end = find_block_end(lines, i)
            result.extend(lines[i : pane_end + 1])
            i = pane_end + 1
            continue

        # Split group (container pane with children, no command) — recurse
        if is_split_group(line):
            block_end = find_block_end(lines, i)
            result.append(line)
            inner = process_tab_block(lines, i + 1, block_end - 1, ros)
            result.extend(inner)
            result.append(lines[block_end])
            i = block_end + 1
            continue

        # Bare pane (no command, no block) — convert to bash pane
        if is_bare_pane(line) and "{" not in line:
            result.extend(convert_bare_pane(line, ros))
            i += 1
            continue

        # Bare pane with empty block { } — convert to bash pane
        if is_bare_pane(line) and "{" in line:
            pane_end = find_block_end(lines, i)
            result.extend(convert_bare_pane(line, ros))
            i = pane_end + 1
            continue

        result.append(line)
        i += 1

    return result


def process_layout(text):
    lines = text.split("\n")
    result = []
    i = 0

    while i < len(lines):
        line = lines[i]

        # Leave swap/template blocks untouched
        if re.match(
            r"\s*(swap_tiled_layout|swap_floating_layout|new_tab_template)\b", line
        ):
            block_end = find_block_end(lines, i)
            result.extend(lines[i : block_end + 1])
            i = block_end + 1
            continue

        # Process real tab blocks
        name_match = re.search(r'name="([^"]+)"', line)
        if re.match(r"\s*tab\b", line) and name_match and "{" in line:
            tab_name = name_match.group(1)
            ros = ROS_TAB_CONFIG.get(tab_name, DEFAULT_ROS_ENABLE)
            tab_end = find_block_end(lines, i)

            result.append(line)
            inner = process_tab_block(lines, i + 1, tab_end - 1, ros)
            result.extend(inner)
            result.append(lines[tab_end])
            i = tab_end + 1
            continue

        result.append(line)
        i += 1

    return "\n".join(result)


def backup(filepath):
    if os.path.exists(filepath):
        bak = filepath + ".bak"
        with open(filepath) as f:
            content = f.read()
        with open(bak, "w") as f:
            f.write(content)
        print(f"Backed up → {bak}")


def main():
    print("Dumping layout...")
    layout = dump_layout()

    print("Processing panes...")
    merged = process_layout(layout)

    backup(LAYOUT_FILE)

    with open(LAYOUT_FILE, "w") as f:
        f.write(merged)

    print(f"Saved → {LAYOUT_FILE}")
    print("\nROS_ENABLE config:")
    for tab, val in ROS_TAB_CONFIG.items():
        print(f"  {tab}: {val}")
    print(f"  (all others): {DEFAULT_ROS_ENABLE}")


if __name__ == "__main__":
    main()
