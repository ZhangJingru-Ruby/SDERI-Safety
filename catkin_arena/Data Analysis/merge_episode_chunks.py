#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Merge several recorder output folders into one continuous raw-data folder.

Use case:
    The simulator may hang after about 30 episodes, so run it in smaller
    chunks (for example 25 episodes each), then merge those chunks into one
    folder with episode numbers 0..99 before running batch_generate_metrics.py.

The script keeps raw chunk folders unchanged. It filters by episode.csv row
positions so scan/odom/cmd_vel/start_goal stay aligned with episode.csv, then
renumbers episode ids continuously.
"""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

REQUIRED_FILES = [
    "cmd_vel.csv",
    "episode.csv",
    "odom.csv",
    "params.yaml",
    "scan.csv",
    "start_goal.csv",
]

CSV_FILES = ["cmd_vel.csv", "episode.csv", "odom.csv", "scan.csv", "start_goal.csv"]
OPTIONAL_EPISODE_CSV_FILES = ["safety_throttle_debug.csv"]


def read_csv(path: Path) -> tuple[list[str], list[list[str]]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return [], []
    return rows[0], rows[1:]


def write_csv(path: Path, header: list[str], rows: list[list[str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def parse_episode(value: str) -> int | None:
    try:
        return int(float(str(value).strip()))
    except Exception:
        return None


def validate_chunk(path: Path) -> None:
    missing = [name for name in REQUIRED_FILES if not (path / name).exists()]
    if missing:
        raise FileNotFoundError(f"{path} is missing required files: {', '.join(missing)}")


def usable_episode_ids(
    episode_rows: list[list[str]],
    episode_col: int,
    min_rows_per_episode: int,
    exclude_highest_episode: bool,
) -> list[int]:
    counts: dict[int, int] = {}
    for row in episode_rows:
        if episode_col >= len(row):
            continue
        ep = parse_episode(row[episode_col])
        if ep is None:
            continue
        counts[ep] = counts.get(ep, 0) + 1

    ids = [ep for ep, count in counts.items() if count >= min_rows_per_episode]
    if exclude_highest_episode and ids:
        ids = [ep for ep in ids if ep < max(ids)]
    return sorted(ids)


def build_row_plan(
    episode_rows: list[list[str]],
    episode_col: int,
    episode_ids: list[int],
    next_episode: int,
    remaining: int,
) -> tuple[list[int], dict[int, int], int]:
    selected_old_ids = episode_ids[:remaining]
    ep_map = {old: next_episode + idx for idx, old in enumerate(selected_old_ids)}

    row_indices = []
    for idx, row in enumerate(episode_rows):
        if episode_col >= len(row):
            continue
        old_ep = parse_episode(row[episode_col])
        if old_ep in ep_map:
            row_indices.append(idx)

    return row_indices, ep_map, len(selected_old_ids)


def copy_filtered_csv(
    src: Path,
    dst: Path,
    row_indices: list[int],
    ep_map: dict[int, int],
) -> int:
    header, rows = read_csv(src)
    if not header:
        write_csv(dst, header, [])
        return 0

    ep_col = header.index("episode") if "episode" in header else None
    out_rows = []
    for idx in row_indices:
        if idx >= len(rows):
            continue
        row = list(rows[idx])
        if ep_col is not None and ep_col < len(row):
            old_ep = parse_episode(row[ep_col])
            if old_ep not in ep_map:
                continue
            row[ep_col] = str(ep_map[old_ep])
        out_rows.append(row)

    if dst.exists():
        _, existing_rows = read_csv(dst)
        out_rows = existing_rows + out_rows

    write_csv(dst, header, out_rows)
    return len(out_rows)


def copy_filtered_episode_csv(
    src: Path,
    dst: Path,
    ep_map: dict[int, int],
) -> int:
    header, rows = read_csv(src)
    if not header:
        write_csv(dst, header, [])
        return 0
    if "episode" not in header:
        raise ValueError(f"{src} has no episode column")

    ep_col = header.index("episode")
    out_rows = []
    for row in rows:
        row = list(row)
        if ep_col >= len(row):
            continue
        old_ep = parse_episode(row[ep_col])
        if old_ep not in ep_map:
            continue
        row[ep_col] = str(ep_map[old_ep])
        out_rows.append(row)

    if dst.exists():
        _, existing_rows = read_csv(dst)
        out_rows = existing_rows + out_rows

    write_csv(dst, header, out_rows)
    return len(out_rows)


def merge_chunks(
    chunks: list[Path],
    output: Path,
    target_episodes: int,
    min_rows_per_episode: int,
    exclude_highest_episode: bool,
    force: bool,
) -> int:
    if output.exists():
        if not force:
            raise FileExistsError(f"output exists: {output}. Use --force to overwrite.")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)

    next_episode = 0
    for chunk in chunks:
        validate_chunk(chunk)
        episode_header, episode_rows = read_csv(chunk / "episode.csv")
        if "episode" not in episode_header:
            raise ValueError(f"{chunk / 'episode.csv'} has no episode column")

        episode_col = episode_header.index("episode")
        ids = usable_episode_ids(
            episode_rows,
            episode_col,
            min_rows_per_episode=min_rows_per_episode,
            exclude_highest_episode=exclude_highest_episode,
        )
        if not ids:
            print(f"[WARN] no usable complete episodes in {chunk}")
            continue

        remaining = target_episodes - next_episode
        if remaining <= 0:
            break

        row_indices, ep_map, added = build_row_plan(
            episode_rows,
            episode_col,
            ids,
            next_episode=next_episode,
            remaining=remaining,
        )

        if added <= 0:
            continue

        print(
            f"[MERGE] {chunk.name}: old episodes {ids[:added][0]}..{ids[:added][-1]} "
            f"-> new episodes {next_episode}..{next_episode + added - 1}"
        )

        for name in CSV_FILES:
            copy_filtered_csv(chunk / name, output / name, row_indices, ep_map)

        for name in OPTIONAL_EPISODE_CSV_FILES:
            optional_src = chunk / name
            if optional_src.exists():
                copy_filtered_episode_csv(optional_src, output / name, ep_map)

        if next_episode == 0:
            shutil.copy2(chunk / "params.yaml", output / "params.yaml")

        next_episode += added

    if next_episode < target_episodes:
        print(f"[WARN] merged only {next_episode}/{target_episodes} episodes.")
    else:
        print(f"[OK] merged {next_episode} episodes into {output}")

    return next_episode


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Merge multiple arena-evaluation recorder folders into one continuous raw-data folder."
    )
    parser.add_argument("chunks", nargs="+", help="recorder chunk folders, in run order")
    parser.add_argument("--output", "-o", required=True, help="merged output folder")
    parser.add_argument("--target-episodes", "-n", type=int, default=100)
    parser.add_argument("--min-rows-per-episode", type=int, default=6)
    parser.add_argument(
        "--include-highest-episode",
        action="store_true",
        help="include the highest episode id in each chunk. Default excludes it because it is often partial.",
    )
    parser.add_argument("--force", action="store_true", help="overwrite output folder if it exists")
    args = parser.parse_args()

    chunks = [Path(p).expanduser().resolve() for p in args.chunks]
    output = Path(args.output).expanduser().resolve()

    merge_chunks(
        chunks=chunks,
        output=output,
        target_episodes=args.target_episodes,
        min_rows_per_episode=args.min_rows_per_episode,
        exclude_highest_episode=not args.include_highest_episode,
        force=args.force,
    )


if __name__ == "__main__":
    main()
