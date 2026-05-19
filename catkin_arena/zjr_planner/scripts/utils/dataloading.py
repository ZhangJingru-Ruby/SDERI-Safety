from __future__ import annotations

import json
from pathlib import Path
from typing   import Dict, List, Tuple

import cv2
import numpy as np
import torch

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #
MAP_SIZE          = 100       # map is 100 × 100 cells
MAP_RES           = 0.05      # 1 cell = 5 cm
N_BANDS           = 10
SAFE_DIST_M       = 0.30      # metres – obstacle inflation
BOUNDS_WIDTH_M    = 1.00      # inner dead-zone radius around robot
NEAR_PATH_RADIUS  = 25        # pixels – corridor half-width for valid cells
DEVICE_DEFAULT    = torch.device("cpu")


# --------------------------------------------------------------------------- #
# Rule-based slim pre-processing
# --------------------------------------------------------------------------- #

def compute_front_sector_rho(
    occ_map: np.ndarray,
    start_pt: Dict,
    goal_pt: Dict,
    radius_m: float = 2.0,
    fov_deg: float = 90.0,
    inner_m: float = 0.30,
    rho_ref: float = 0.15,
) -> Tuple[float, float]:
    """
    Compute obstacle density in a goal-facing front sector.

    Returns:
        rho_scaled: clipped normalized density in [0, 1]
        rho_raw   : raw occupied ratio in the sector

    Notes:
        - This uses start->goal direction as the "front" direction.
        - It does not require robot yaw, so we don't need to touch main.py yet.
        - occ_map occupied semantics: occ_map > 0 means occupied.
    """
    occ = np.asarray(occ_map)
    if occ.size == 0:
        return 0.0, 0.0

    occupied = occ > 0

    center = MAP_SIZE // 2
    mx_grid, my_grid = np.indices(occ.shape)

    dx = (mx_grid - center) * MAP_RES
    dy = (my_grid - center) * MAP_RES
    r = np.sqrt(dx * dx + dy * dy)

    # front direction: current start -> final goal
    fx = float(goal_pt["x"]) - float(start_pt["x"])
    fy = float(goal_pt["y"]) - float(start_pt["y"])
    fn = np.sqrt(fx * fx + fy * fy)

    if fn < 1e-6:
        # fallback direction: +x in local map
        fx, fy = 1.0, 0.0
    else:
        fx, fy = fx / fn, fy / fn

    cos_angle = (dx * fx + dy * fy) / (r + 1e-6)
    cos_thresh = np.cos(np.deg2rad(fov_deg * 0.5))

    sector = (
        (r >= inner_m) &
        (r <= radius_m) &
        (cos_angle >= cos_thresh)
    )

    denom = int(sector.sum())
    if denom <= 0:
        return 0.0, 0.0

    rho_raw = float(occupied[sector].sum()) / float(denom)

    # Gentle normalization. rho_ref=0.15 means:
    # raw density 0.15 or above maps to 1.0.
    rho_scaled = float(np.clip(rho_raw / max(rho_ref, 1e-6), 0.0, 1.0))

    return rho_scaled, rho_raw


def preprocess_for_rule(scenario: Dict, device = DEVICE_DEFAULT) -> Tuple[np.ndarray, List[List[List[float]]], Dict, torch.Tensor]:
    """
    Returns the four items the rule API expects:

        band_map         (100,100)  numpy int
        paths_positions  list[3][T][2]  original waypoints
        start_pt         dict {"x":..,"y":..}
        tau              (1,1) torch float   distance_hat ∈ [0,1]
    """
    occ_map   = scenario["occupancy_map"]
    start_pt  = scenario["start_point"]
    goal_pt   = scenario["goal_point"]
    paths     = preprocess_paths(scenario["path_dicts"])     # ensure 3 paths

    print("[geom-debug] start_pt:", start_pt)
    print("[geom-debug] goal_pt :", goal_pt)

    for i, p in enumerate(paths):
        path = p.get("path", [])
        print("[geom-debug] path", i, "len:", len(path))
        if path:
            first = path[0]["position"]
            last = path[-1]["position"]
            print("[geom-debug] path", i, "first:", first, "last:", last)

            d0 = np.linalg.norm([
                float(first[0]) - float(start_pt["x"]),
                float(first[1]) - float(start_pt["y"])
            ])
            print("[geom-debug] path", i, "dist(first,start):", float(d0))

    # 1) build band map  (reuse helper)
    _, band_idx, _ = build_valid_mask_and_bands(occ_map, paths, start_pt)
    band_map = band_idx.cpu().numpy()        # (100,100) int

    # 2) τ  (scaled 0-1, clipped)
    dist = np.linalg.norm([(goal_pt["x"] - start_pt["x"]),
                           (goal_pt["y"] - start_pt["y"])])       # metres
    d_hat = np.clip(dist / 25.0, 0.0, 1.0)
    tau   = torch.tensor([[d_hat]], dtype=torch.float32, device=device)  # (1,1)

    # 3) raw path positions (for overlay)
    paths_pos = [[pt["position"] for pt in p["path"]] for p in paths]

    rho_front, rho_front_raw = compute_front_sector_rho(
        occ_map,
        start_pt,
        goal_pt,
        radius_m=2.0,
        fov_deg=90.0,
        inner_m=0.30,
        rho_ref=0.15,
    )

    print("[rho-debug] rho_front_scaled=%.4f rho_front_raw=%.4f" %
        (rho_front, rho_front_raw))

    return band_map, paths_pos, start_pt, tau, rho_front


# --------------------------------------------------------------------------- #
# Raw-JSON helpers
# --------------------------------------------------------------------------- #
def load_raw_json(path: str | Path) -> Dict:
    """Load the raw scenario JSON exactly as on disk."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r") as f:
        return json.load(f)


def scenario_from_json(raw: Dict) -> Dict:
    """
    Re-shape the odd key pattern of the recording script into a cleaner dict.

    Compatible with:
        path_1_1_1 / path_1_1_2 / path_1_1_3
        path_1_1_0 / path_1_1_1 / path_1_1_2
        fewer than 3 paths, e.g. only path_1_1_1 and path_1_1_2
    """
    occ_map = np.array(raw["pointcloud1"]["grid_map"], dtype=np.uint8)
    start   = raw["start_point1"]
    goal    = raw["goal_points_1_1"]

    # 1. 自动收集所有 path_1_1_*，不要硬编码必须有 _1/_2/_3
    path_keys = [
        k for k in raw.keys()
        if k.startswith("path_1_1_")
    ]

    def _path_key_index(k: str) -> int:
        try:
            return int(k.split("_")[-1])
        except Exception:
            return 999999

    path_keys = sorted(path_keys, key=_path_key_index)

    if not path_keys:
        raise KeyError(
            "No path_1_1_* keys found in raw json. Available keys: %s"
            % list(raw.keys())
        )

    paths = [raw[k] for k in path_keys]

    # 2. 兼容只有 1 或 2 条 path 的情况，复制最后一条补到 3 条
    if len(paths) < 3:
        print("[dataloading] Only %d path(s) found: %s" % (len(paths), path_keys))
        print("[dataloading] Duplicating last path to make exactly 3 paths.")
        paths = paths + [paths[-1]] * (3 - len(paths))
    elif len(paths) > 3:
        print("[dataloading] %d paths found: %s" % (len(paths), path_keys))
        print("[dataloading] Using first 3 paths.")
        paths = paths[:3]

    print("[dataloading] using path keys:", path_keys[:3])

    return dict(
        occupancy_map = occ_map,
        start_point   = start,
        goal_point    = goal,
        path_dicts    = paths,
    )

# --------------------------------------------------------------------------- #
# Path, mask & band helpers
# --------------------------------------------------------------------------- #
def preprocess_paths(paths: List[Dict], target_num: int = 3) -> List[Dict]:
    """
    Ensure exactly `target_num` paths by duplicating or truncating.

    This keeps shapes fixed downstream.
    """
    if len(paths) < target_num:
        paths += [paths[-1]] * (target_num - len(paths))
    elif len(paths) > target_num:
        paths = paths[:target_num]
    return paths


def draw_path_lines(paths   : List[Dict],
                    start_pt: Dict) -> np.ndarray:
    """
    Rasterise all paths into a binary mask - path pixels = 1, else 0.
    """
    mask = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)

    for p_dict in paths:
        pts_map: List[Tuple[int, int]] = [
            world_to_map_coords(
                pt["position"][0], pt["position"][1],
                start_pt["x"],      start_pt["y"],
                MAP_RES)
            for pt in p_dict["path"]
        ]
        for i in range(len(pts_map) - 1):
            cv2.line(mask,
                     pts_map[i], pts_map[i + 1],
                     color=1, thickness=1)

    return mask


def compute_path_dist_map_fast(occupancy_map: np.ndarray,
                               paths        : List[Dict],
                               start_pt     : Dict) -> np.ndarray:
    """
    Return a float32 (100, 100) array - for each free cell, the Euclidean
    distance (metres) to the nearest rasterised global-path pixel.
    """
    # 1) build path pixels = 0 mask
    mask = np.ones_like(occupancy_map, dtype=np.uint8)
    path_pix = draw_path_lines(paths, start_pt)
    mask[path_pix == 1] = 0          # 0 where path

    # 2) distance transform in pixels
    dist_pix = cv2.distanceTransform(mask, cv2.DIST_L2, 5)

    # 3) convert to metres
    return dist_pix.astype(np.float32) * MAP_RES


def build_valid_mask_and_bands(occ_map    : np.ndarray,
                               paths      : List[Dict],
                               start_pt   : Dict,
                               safe_dist_m: float       = SAFE_DIST_M,
                               bounds_m   : float       = BOUNDS_WIDTH_M,
                               device     = DEVICE_DEFAULT
                               ) -> Tuple[torch.Tensor, torch.Tensor, np.ndarray]:
    """
    Core logic reused from training:

        • valid_mask      : (100,100) bool
        • band_idx_map    : (100,100) long  (-1 = invalid)
        • band_mean_dist  : (10,)     float np.ndarray

    Debug version:
        Prints occupancy-map semantics, path drawing status,
        mask survival counts after each filtering step,
        and final band distribution.
    """

    def _mask_count(name, mask):
        print("[mask-debug] %-28s valid=%d / %d" % (
            name, int(mask.sum()), mask.size
        ))

    print("\n========== [mask-debug] build_valid_mask_and_bands ==========")
    print("[mask-debug] occ_map shape:", occ_map.shape)
    print("[mask-debug] start_pt:", start_pt)
    print("[mask-debug] safe_dist_m:", safe_dist_m)
    print("[mask-debug] bounds_m:", bounds_m)
    print("[mask-debug] NEAR_PATH_RADIUS:", NEAR_PATH_RADIUS)
    print("[mask-debug] MAP_SIZE:", MAP_SIZE, "MAP_RES:", MAP_RES)

    try:
        ov, oc = np.unique(occ_map, return_counts=True)
        print("[mask-debug] occ unique:", list(zip(ov.tolist(), oc.tolist()))[:20])
    except Exception as e:
        print("[mask-debug] failed to inspect occ_map:", e)

    try:
        print("[mask-debug] num paths:", len(paths))
        for i, p_dict in enumerate(paths):
            path = p_dict.get("path", [])
            print("[mask-debug] path %d len: %d" % (i, len(path)))
            if len(path) > 0:
                first = path[0].get("position", path[0])
                last  = path[-1].get("position", path[-1])
                print("[mask-debug] path %d first: %s" % (i, str(first)))
                print("[mask-debug] path %d last : %s" % (i, str(last)))

                try:
                    d0 = np.linalg.norm([
                        float(first[0]) - float(start_pt["x"]),
                        float(first[1]) - float(start_pt["y"])
                    ])
                    print("[mask-debug] path %d dist(first,start): %.3f" % (i, d0))
                except Exception as e:
                    print("[mask-debug] path %d cannot compute dist(first,start): %s" % (i, e))
    except Exception as e:
        print("[mask-debug] failed to inspect paths:", e)

    # --------- 1. path distance map ------------------------------
    path_pix = draw_path_lines(paths, start_pt)
    print("[mask-debug] path pixels:", int(path_pix.sum()))

    dist_map = compute_path_dist_map_fast(occ_map, paths, start_pt)  # metres

    try:
        print("[mask-debug] dist_map min/max/mean:",
              float(np.min(dist_map)),
              float(np.max(dist_map)),
              float(np.mean(dist_map)))
    except Exception as e:
        print("[mask-debug] failed to inspect dist_map:", e)

    # --------- 2. rule-based masks -------------------------------
    valid_mask = np.ones_like(occ_map, dtype=bool)
    _mask_count("initial", valid_mask)

    # 2a. boundary dead-zone
    centre_px = MAP_SIZE // 2
    bounds_px = int((MAP_SIZE * MAP_RES / 2 - bounds_m) / MAP_RES)
    lo, hi    = centre_px - bounds_px, centre_px + bounds_px

    print("[mask-debug] bounds_px:", bounds_px, "lo:", lo, "hi:", hi)

    valid_mask[lo:hi, lo:hi] = False
    _mask_count("after bounds", valid_mask)

    # 2b. obstacle inflation
    if safe_dist_m > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (int(2 * safe_dist_m / MAP_RES + 1),
             int(2 * safe_dist_m / MAP_RES + 1))
        )

        print("[mask-debug] obstacle kernel shape:", kernel.shape)

        dilated = cv2.dilate(occ_map.astype(np.uint8), kernel)

        try:
            dv, dc = np.unique(dilated, return_counts=True)
            print("[mask-debug] dilated unique:", list(zip(dv.tolist(), dc.tolist()))[:20])
        except Exception as e:
            print("[mask-debug] failed to inspect dilated occ:", e)

        valid_mask[dilated > 0] = False

    _mask_count("after obstacle inflation", valid_mask)

    # 2c. near-path corridor
    path_corridor = cv2.dilate(
        path_pix,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * NEAR_PATH_RADIUS + 1,
             2 * NEAR_PATH_RADIUS + 1)
        )
    )

    print("[mask-debug] corridor pixels:", int((path_corridor > 0).sum()))

    valid_mask[path_corridor == 0] = False
    _mask_count("after path corridor", valid_mask)

    # --------- 3. band index map & means -------------------------
    band_idx_map = np.full_like(occ_map, fill_value=-1, dtype=np.int64)

    dist_valid = dist_map[valid_mask]

    print("[mask-debug] dist_valid size:", int(dist_valid.size))

    if dist_valid.size == 0:
        print("[mask-debug] WARNING: dist_valid is empty. No valid cells survived all masks.")
        max_d = 1e-3
    else:
        max_d = float(dist_valid.max())

    print("[mask-debug] max_d:", max_d)

    band_width = max_d / N_BANDS if N_BANDS > 0 else max_d
    print("[mask-debug] band_width:", band_width)

    for b in range(N_BANDS):
        lo_d, hi_d = b * band_width, (b + 1) * band_width
        band_cells = (dist_map >= lo_d) & (dist_map < hi_d) & valid_mask
        band_idx_map[band_cells] = b
        print("[mask-debug] band %d range [%.4f, %.4f) cells=%d" %
              (b, lo_d, hi_d, int(band_cells.sum())))

    vals, counts = np.unique(band_idx_map, return_counts=True)
    print("[mask-debug] band_idx unique:", list(zip(vals.tolist(), counts.tolist())))

    # mean distance per band
    band_mean_dist = np.zeros(N_BANDS, dtype=np.float32)
    for b in range(N_BANDS):
        d = dist_map[band_idx_map == b]
        band_mean_dist[b] = d.mean() if d.size else 0.0

    print("[mask-debug] band_mean_dist:", band_mean_dist.tolist())
    print("========== [mask-debug] end build_valid_mask_and_bands ==========\n")

    # convert to torch
    valid_mask_t   = torch.from_numpy(valid_mask)
    band_idx_map_t = torch.from_numpy(band_idx_map)

    return valid_mask_t, band_idx_map_t, band_mean_dist


# --------------------------------------------------------------------------- #
# Coords transformation
# --------------------------------------------------------------------------- #
def world_to_map_coords(g_x: float, g_y: float, start_x: float, start_y: float, map_resolution: float = 0.05):
    map_size = 100
    # 计算地图左下角原点（世界坐标）
    origin_x = start_x - (map_size // 2) * map_resolution  # start_x - 2.5 meters
    origin_y = start_y - (map_size // 2) * map_resolution  # start_y - 2.5 meters
    
    # 转换为地图索引（左下角为原点）
    mx = int((g_x - origin_x) / map_resolution)
    my = int((g_y - origin_y) / map_resolution)
    
    return mx, my


def map_to_world_coords(mx: int, my: int, start_x: float, start_y: float, map_resolution: float = 0.05) :
    map_size = 100
    half_size = map_size // 2
    # 计算地图左下角原点（世界坐标）
    origin_x = start_x - half_size * map_resolution  # start_x - 2.5 meters
    origin_y = start_y - half_size * map_resolution  # start_y - 2.5 meters
    
    # 计算实际世界坐标
    g_x = origin_x + mx * map_resolution
    g_y = origin_y + my * map_resolution
    
    return g_x, g_y

# --------------------------------------------------------------------------- #
# 🌟  Quick CLI check (optional)
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    # Rudimentary sanity-run
    json_path = Path(__file__).parent / "data" / "example_data.json"
    raw       = load_raw_json(json_path)
    scenario  = scenario_from_json(raw)
    band_map, paths_pos, start_pt, tau = preprocess_for_rule(scenario)
    print("dist_goal:", dgoal.item())
    print("✅ dataloading.py loaded. Has preprocess_for_rule =", 'preprocess_for_rule' in dir())
