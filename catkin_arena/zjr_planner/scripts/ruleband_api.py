"""
rule_api.py
===========
Rule-based sub-goal generator – no neural net, no checkpoint.
Steps
-----
1.  Load raw scenario (JSON) or pre-built tensors via utils.data_loader.
2.  Compute **τ = dist_goal / MAX_DIST**  (auto-normalised per batch).
3.  Convert τ → probability vector over 10 bands with `rule_probs_temp`.
4.  Sample a band, pick a random cell inside, map→world, done.
5.  Optional: `debug=True` shows heat-map + chosen cell.
"""

from __future__ import annotations
from pathlib import Path
import random, warnings
from typing import Tuple, List

import torch, numpy as np
import os

from utils.dataloading import (load_raw_json, scenario_from_json, preprocess_for_rule)
from utils.environment_functions import visualize_band_and_subgoal, map_to_world_coords

# ------------------------------------------------------------
N_BANDS = 10
MAX_GOAL_DIST = 15.0          # metres – cap for τ normalisation
# ------------------------------------------------------------

# ─────────────────────────  RULE  ⟶  P(band)  ──────────────────────────
def rule_probs_temp(tau: torch.Tensor,
                    N: int = N_BANDS,
                    T_min: float = 0.05,
                    T_max: float = 6.0) -> torch.Tensor:
    """
    tau may be shape (B,) or (B,1) – we squeeze so output is (B,N).
    """
    tau = tau.reshape(-1)                    # <-- squeeze any trailing dims
    idx = torch.arange(N, device=tau.device) # (N,)
    T   = T_min + tau.unsqueeze(-1) * (T_max - T_min)  # (B,1)
    logits = -idx / T                                   # (B,N)
    return torch.softmax(logits, dim=-1)                # (B,N)


# ─────────────────────────  cell sampler  ──────────────────────────────
def _sample_cell_from_band(band_map: np.ndarray, band_idx: int) -> Tuple[int, int]:
    cand = np.argwhere(band_map == band_idx)          # (K,2)
    if cand.size == 0:                                # graceful fallback
        for delta in range(1, N_BANDS):
            for alt in (band_idx-delta, band_idx+delta):
                if 0 <= alt < N_BANDS:
                    cand = np.argwhere(band_map == alt)
                    if cand.size:
                        warnings.warn(f"Band {band_idx} empty; using {alt}", RuntimeWarning)
                        band_idx = alt
                        break
            if cand.size:
                break
    if cand.size == 0:
        raise RuntimeError("No valid cells in any band.")
    mx, my = cand[random.randrange(len(cand))]
    return int(mx), int(my), int(band_idx)            # return final band too

# ─────────────────────────  API CLASS  ─────────────────────────────────
class RuleBandAPI:
    """
    Drop-in replacement for neural BandNetAPI but 100 % rule-based.
    """

    def __init__(self, device: str = "cpu"):
        self.device = torch.device(device)

    # ---------- top-level helpers ----------
    def predict_from_file(self, json_path: str | Path,
                          debug: bool=False) -> Tuple[float, float]:
        raw = load_raw_json(json_path)
        scenario = scenario_from_json(raw)
        return self.predict_from_scenario(scenario, debug=debug)

    def predict_from_scenario(self, scenario: dict,
                              debug: bool=False) -> Tuple[float, float]:
        out = preprocess_for_rule(scenario)

        if len(out) == 5:
            band_map, paths_pos, start_pt, tau, rho_front = out
        else:
            band_map, paths_pos, start_pt, tau = out
            rho_front = 0.0

        return self.predict(band_map, paths_pos, start_pt, tau, rho_front=rho_front, debug=debug)


    # ---------- core ----------
    @torch.inference_mode()
    def predict(self,
                band_map : np.ndarray,
                paths_pos: List,
                start_pt : dict,
                tau      : torch.Tensor,
                rho_front: float = 0.0,
                debug: bool=False
            ) -> Tuple[float, float, float, int]:
        """
        tau      : (1,1) torch float in [0,1]
        band_map : (100,100) int (-1 or 0..9)

        Current restored SDERI version:
            Use tau as a temporary ERI proxy:
                eri_rule = tau

        This lets the same scalar control:
            1. subgoal distribution entropy
            2. adaptive replanning time in main.py
        """

        # 0. Current minimal ERI proxy
        tau_scalar = float(tau.reshape(-1)[0].clamp(0.0, 1.0).item())

        rho_front = float(np.clip(rho_front, 0.0, 1.0))

        # Default = 0.0, meaning rho is monitored but does not affect behavior.
        rho_gain = float(os.environ.get("SDERI_RHO_GAIN", "0.0"))
        rho_deadband = float(os.environ.get("SDERI_RHO_DEADBAND", "0.20"))

        rho_extra = rho_gain * max(0.0, rho_front - rho_deadband)

        eri_rule = float(np.clip(tau_scalar + rho_extra, 0.0, 1.0))

        # 1. softmax over bands
        # Keep current behavior: tau/eri controls temperature
        probs = rule_probs_temp(torch.tensor([[eri_rule]], dtype=torch.float32, device=self.device))

        # 2. sample band
        band_idx = int(torch.multinomial(probs, 1)[0].item())

        # 3. sample cell & map→world
        mx, my, band_idx_final = _sample_cell_from_band(band_map, band_idx)
        wx, wy = map_to_world_coords(mx, my, start_pt["x"], start_pt["y"])

        # 4. lightweight debug
        print(
            "[ruleband] tau=%.3f rho_front=%.3f rho_gain=%.3f eri_rule=%.3f "
            "sampled band=%d, final band=%d, subgoal=(%.3f, %.3f)" %
            (tau_scalar, rho_front, rho_gain, eri_rule, band_idx, band_idx_final, wx, wy)
        )

        return float(wx), float(wy), float(eri_rule), int(band_idx_final)

# ─────────────────────────  CLI DEMO  ──────────────────────────────────
if __name__ == "__main__":
    api = RuleBandAPI()
    x, y = api.predict_from_file("data/example_data.json", debug=True)
    print(f"Rule sub-goal: ({x:.3f}, {y:.3f})")