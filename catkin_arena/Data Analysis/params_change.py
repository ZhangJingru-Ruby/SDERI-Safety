#!/usr/bin/env python3
# filename: params_change.py
"""
递归查找并直接覆盖修改 params.yaml 中的 local_planner：
 - "teb" -> "DRL"
 - "rosnav" -> "subgoal_supply"

默认直接修改（不备份）。如果想先看 dry-run，可传 --dry-run。
"""

from pathlib import Path
import argparse
import yaml
import sys

def normalize_value(v):
    if v is None:
        return ""
    return str(v).strip().lower()

def should_transform(value):
    v = normalize_value(value)
    if v == "teb":
        return "drl"
    if v == "rosnav":
        return "subgoal_supply"
    return None

def traverse_and_replace(obj, debug=False):
    changed = False
    count = 0
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "local_planner":
                if debug:
                    print(f"  DEBUG: found local_planner = {repr(v)}")
                new_val = should_transform(v)
                if new_val is not None:
                    if v != new_val:
                        obj[k] = new_val
                        changed = True
                        count += 1
            else:
                c, n = traverse_and_replace(v, debug=debug)
                if c:
                    changed = True
                    count += n
    elif isinstance(obj, list):
        for idx, item in enumerate(obj):
            c, n = traverse_and_replace(item, debug=debug)
            if c:
                changed = True
                count += n
    return changed, count

def process_file(path: Path, dry_run=False, debug=False):
    try:
        text = path.read_text(encoding="utf-8")
    except Exception as e:
        return False, f"read_error: {e}"

    try:
        data = yaml.safe_load(text)
    except Exception as e:
        return False, f"yaml_parse_error: {e}"

    if data is None:
        return False, "empty_yaml"

    changed, count = traverse_and_replace(data, debug=debug)
    if not changed:
        return False, "no_change"

    if dry_run:
        return True, f"would_change {count} entries in {path}"
    try:
        path.write_text(
            yaml.safe_dump(data, default_flow_style=False, sort_keys=False, allow_unicode=True),
            encoding="utf-8"
        )
        return True, f"changed {count} entries in {path}"
    except Exception as e:
        return False, f"write_error: {e}"

def find_params(root: Path):
    for p in root.rglob("params.yaml"):
        yield p

def main():
    p = argparse.ArgumentParser(description="Fix local_planner in params.yaml (no backup by default).")
    p.add_argument("--root", "-r", default=".", help="root folder to search (default current dir)")
    p.add_argument("--dry-run", action="store_true", help="only show which files would be changed, do not write")
    p.add_argument("--debug", action="store_true", help="print all found local_planner values")
    args = p.parse_args()

    root = Path(args.root).resolve()
    if not root.exists():
        print("root path not found:", root)
        sys.exit(1)

    files = list(find_params(root))
    if not files:
        print("No params.yaml under", root)
        return

    total_changed = 0
    total_files = 0
    for f in files:
        if args.debug:
            print(f"\n[CHECK] {f}")
        ok, msg = process_file(f, dry_run=args.dry_run, debug=args.debug)
        total_files += 1
        if ok:
            print("[OK] ", msg)
            if not args.dry_run:
                total_changed += 1
        else:
            print("[SKIP]", f, "->", msg)
    print(f"\nScanned {total_files} files. Files modified: {total_changed} (dry_run={args.dry_run})")

if __name__ == "__main__":
    main()

