"""
Which sensed MoCap marker is which designed marker of a bar, and the bar pose that fits them.

Motive's marker ids are not used (they shift). The old monitor's matcher (bar_mocap_fit.py = BMF), ported unchanged;
numpy only.

- `match_and_fit(sensed, design)`: the full search, every design triangle against every sensed triple.
- `fit_near_hint(sensed, design, T_hint)`: the quick match around the last pose while following a bar live.
- `BarFit`: what either returns; `ok` False carries a plain-language `reason`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
from math import ceil

import numpy as np

from .bar_pose import kabsch, transform_points

# * A design triangle used to start a match must be at least this high (its smallest altitude). Balls with the
#   same label on joints with the same roll lie on one line along the bar, and three points on a line leave the
#   turn about that line open. A pair plus any far ball is at least 25 mm high. (BMF:62)
MIN_TRIANGLE_HEIGHT_M = 0.015

# * A sensed and a placed design marker closer than this match (4 mm: ten times below the 48 mm pair spacing, above
#   Motive's ~0.8 mm noise). (BMF:66)
MATCH_TOL_M = 0.004
# * Following live: a sensed marker this close to a design marker placed at the last pose is taken as that marker.
#   20 mm is well under half the 48 mm pair spacing, so a ball can never be taken for its neighbour. (BMF:70)
TRACK_GATE_M = 0.02

# * Motive markers we do not trust: a fit residual above 3 mm, or a ball size far from the 19 mm balls on the plates
#   (a reflection, a merged blob). (BMF:74-78)
MAX_MARKER_ERROR_M = 0.003
MARKER_SIZE_RANGE_M = (0.012, 0.026)


@dataclass
class BarFit:
    """The result of matching sensed markers to one bar's design markers.

    A "sensed index" is a row of the sensed points handed to the fit; a "design index" is a row of the design points.

    Attributes:
        ok: True when enough markers matched and no other assignment fits as well.
        reason: Why the fit is not ok, in plain language; empty when ok.
        bar_name: The bar's body id, when known.
        T_world_from_bar: 4x4 pose of the bar frame in world (always a proper rotation, never a mirror). The best
            pose found even when not ok; None when no hypothesis was found at all.
        correspondences: ``{sensed index: design index}`` of the matched markers.
        sensed_ids: Sensed index -> the marker id it came from (Motive's member number, else ``0..N-1``).
        residuals_m: ``{sensed index: distance in metres}`` between each matched sensed marker and its design marker
            placed at the fitted pose.
        rms_m: Root mean square of ``residuals_m``.
        max_residual_m: Largest of ``residuals_m``.
        n_matched: How many markers matched.
        n_design: How many design markers the bar has.
        unmatched_sensed: Sensed indices that match no design marker (strays, another bar's markers).
        missing_design: Design indices that no sensed marker matched (occluded balls).
        ambiguous: A different marker assignment fits equally well.
        second_best: The best assignment that disagrees with the winner: ``{'n_matched', 'rms_m',
            'max_residual_m', 'correspondences'}``; None when there is none.
        n_hypotheses: How many (design triangle, sensed triple) guesses were tried.
        tracked: True when the fit came from the quick match around the last pose (`fit_near_hint`) instead of the
            full search; such a fit skips the ambiguity check (``ambiguous`` False, ``second_best`` None,
            ``n_hypotheses`` 0).
    """

    ok: bool
    reason: str
    bar_name: str = ""
    T_world_from_bar: np.ndarray | None = None
    correspondences: dict = field(default_factory=dict)
    sensed_ids: list = field(default_factory=list)
    residuals_m: dict = field(default_factory=dict)
    rms_m: float | None = None
    max_residual_m: float | None = None
    n_matched: int = 0
    n_design: int = 0
    unmatched_sensed: list = field(default_factory=list)
    missing_design: list = field(default_factory=list)
    ambiguous: bool = False
    second_best: dict | None = None
    n_hypotheses: int = 0
    tracked: bool = False

    def to_json(self) -> dict:
        """The fit as plain lists, dicts and numbers, ready for ``json.dump``.

        Returns:
            dict: Same fields; the pose as a nested 4x4 list (or None).
        """
        T = self.T_world_from_bar
        return {
            "ok": self.ok, "reason": self.reason, "bar_name": self.bar_name,
            "T_world_from_bar": None if T is None else np.asarray(T).tolist(),
            "correspondences": dict(self.correspondences),
            "sensed_ids": list(self.sensed_ids),
            "residuals_m": dict(self.residuals_m),
            "rms_m": self.rms_m, "max_residual_m": self.max_residual_m,
            "n_matched": self.n_matched, "n_design": self.n_design,
            "unmatched_sensed": list(self.unmatched_sensed),
            "missing_design": list(self.missing_design),
            "ambiguous": self.ambiguous, "second_best": self.second_best,
            "n_hypotheses": self.n_hypotheses, "tracked": self.tracked,
        }


# * ================================================================ matcher (BMF:516-833)
#
# * Why the matcher works the way it does (B20 / B21 of 261016_2ModulesDemo_YH):
#   - M1 and M2 of one plate cannot be told apart by distances to the other balls (all ~1 m away, the lists differ
#     by ~0.35 mm). So every guess is judged on ALL markers after the fit: the wrong M1/M2 guess leaves ~48 mm
#     errors elsewhere.
#   - A three-ball triangle cannot reject a mirror (a flipped triangle is a rotated one). Mirrors are rejected by
#     `kabsch` never mirroring plus a minimum of 4 matched balls.
#   - Balls on a line (same label, same roll) are skipped as start triangles.

def _pairwise_distances(pts: np.ndarray) -> np.ndarray:
    """Distance between every two points (BMF:528).

    Args:
        pts: (K, 3) points.

    Returns:
        np.ndarray: (K, K) distances.
    """
    return np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)


def _design_triangles(design: np.ndarray, min_height_m: float) -> list:
    """Every three design markers whose triangle is high enough to fix a pose (BMF:540).

    Args:
        design: (M, 3) design points.
        min_height_m: Smallest allowed altitude of the triangle.

    Returns:
        list[tuple]: ``(a, b, c)`` index triples, a < b < c.
    """
    dist = _pairwise_distances(design)
    triangles = []
    for a, b, c in combinations(range(len(design)), 3):
        twice_area = np.linalg.norm(np.cross(design[b] - design[a], design[c] - design[a]))
        longest = max(dist[a, b], dist[b, c], dist[a, c])
        # The smallest altitude stands on the longest side: 2 * area / longest side.
        if longest > 0.0 and twice_area / longest >= min_height_m:
            triangles.append((a, b, c))
    return triangles


def _assign(sensed: np.ndarray, design_world: np.ndarray, tol_m: float) -> dict:
    """Pair sensed and design markers one-to-one, closest pairs first (BMF:561).

    Args:
        sensed: (N, 3) sensed points.
        design_world: (M, 3) design points already placed in world.
        tol_m: Pairs farther apart than this never match.

    Returns:
        dict: ``{sensed index: design index}``.
    """
    dist = np.linalg.norm(sensed[:, None, :] - design_world[None, :, :], axis=2)
    s_idx, d_idx = np.nonzero(dist < tol_m)
    pairs, used_design = {}, set()
    for k in np.argsort(dist[s_idx, d_idx], kind="stable"):
        s, d = int(s_idx[k]), int(d_idx[k])
        if s not in pairs and d not in used_design:
            pairs[s] = d
            used_design.add(d)
    return pairs


def _fit_pairs(sensed: np.ndarray, design: np.ndarray, pairs: dict) -> np.ndarray:
    """Kabsch on matched pairs (BMF:583).

    Args:
        sensed: (N, 3) sensed points.
        design: (M, 3) design points.
        pairs: ``{sensed index: design index}``, at least 3.

    Returns:
        np.ndarray: 4x4 ``T_world_from_bar``.
    """
    return kabsch(design[list(pairs.values())], sensed[list(pairs.keys())])


def _refine(sensed: np.ndarray, design: np.ndarray, pairs: dict, tol_m: float) -> tuple:
    """Refit on all matched pairs, re-pair once, refit again (BMF:597).

    Args:
        sensed: (N, 3) sensed points.
        design: (M, 3) design points.
        pairs: ``{sensed index: design index}`` of a guess, at least 3.
        tol_m: Match distance.

    Returns:
        tuple: ``(T, pairs, residuals)``: the 4x4 pose, the final pairs (sorted by sensed index) and
        ``{sensed index: residual in metres}``.
    """
    T = _fit_pairs(sensed, design, pairs)
    repaired = _assign(sensed, transform_points(T, design), tol_m)
    # Keep the guess' pairs in the (theoretical) case the refit loses too many.
    if len(repaired) >= 3:
        pairs = repaired
    pairs = dict(sorted(pairs.items()))
    T = _fit_pairs(sensed, design, pairs)
    gaps = np.linalg.norm(transform_points(T, design[list(pairs.values())]) - sensed[list(pairs.keys())], axis=1)
    return T, pairs, {s: float(g) for s, g in zip(pairs, gaps)}


def _rms(residuals: dict) -> float:
    """Root mean square of residuals (BMF:621).

    Args:
        residuals: ``{index: metres}``, not empty.

    Returns:
        float: The rms, metres.
    """
    return float(np.sqrt(np.mean(np.square(list(residuals.values())))))


def _conflicts(a: dict, b: dict) -> bool:
    """Whether two marker assignments disagree about some marker (BMF:633).

    One being a part of the other (a marker more or less) is NOT a disagreement.

    Args:
        a: ``{sensed index: design index}``.
        b: ``{sensed index: design index}``.

    Returns:
        bool: True when a sensed marker goes to different design markers, or a design marker to different sensed
        markers.
    """
    a_by_design = {d: s for s, d in a.items()}
    return any(a.get(s, d) != d or a_by_design.get(d, s) != s for s, d in b.items())


def _default_min_inliers(n_design: int) -> int:
    """How many markers must match for a fit to count, when the caller does not say (BMF:650).

    Args:
        n_design: How many design markers the bar has.

    Returns:
        int: ``max(4, ceil(0.75 * n_design))``, i.e. 6 of 8.
    """
    return max(4, ceil(0.75 * n_design))


def _failed_fit(reason: str, n_sensed: int, n_design: int, n_hypotheses: int = 0) -> BarFit:
    """A fit that found no pose at all (BMF:662).

    Args:
        reason: Why, in plain language.
        n_sensed: Number of sensed markers.
        n_design: Number of design markers.
        n_hypotheses: Guesses tried before giving up.

    Returns:
        BarFit: ``ok=False``, no pose, every marker unmatched.
    """
    return BarFit(ok=False, reason=reason, sensed_ids=list(range(n_sensed)), n_design=n_design,
                  unmatched_sensed=list(range(n_sensed)), missing_design=list(range(n_design)),
                  n_hypotheses=n_hypotheses)


def match_and_fit(sensed_points_m: np.ndarray, design_points_bar_m: np.ndarray, *, tol_m: float = MATCH_TOL_M,
                  side_tol_m: float = 0.002, min_inliers: int | None = None,
                  min_triangle_height_m: float = MIN_TRIANGLE_HEIGHT_M, max_sensed: int = 24) -> BarFit:
    """Find which sensed marker is which design marker and the bar pose that fits them (BMF:679-781).

    Motive's marker ids are not used. Every high-enough design triangle is matched against every sensed triple with
    the same side lengths; each such guess is fitted and judged on ALL markers (most matched, then smallest rms).
    The winner is refitted on all its markers. If a different assignment matches as many markers within ``tol_m``,
    the layout is ambiguous.

    Args:
        sensed_points_m: (N, 3) sensed marker centres in world, metres.
        design_points_bar_m: (M, 3) design marker centres in the bar frame, metres.
        tol_m: A sensed and a placed design marker closer than this match.
        side_tol_m: Triangle sides must agree this well to start a guess (a quick filter only). Raise to 0.003 if
            live fits show residuals above 2 mm.
        min_inliers: Matched markers needed for ``ok``; None means ``max(4, ceil(0.75 * M))``, i.e. 6 of 8.
        min_triangle_height_m: Smallest altitude of a start triangle.
        max_sensed: Refuse more sensed markers than this (time grows fast).

    Returns:
        BarFit: The fit; ``ok`` False with a plain-language ``reason`` when it cannot be trusted.
    """
    sensed = np.asarray(sensed_points_m, dtype=float).reshape(-1, 3)
    design = np.asarray(design_points_bar_m, dtype=float).reshape(-1, 3)
    n_sensed, n_design = len(sensed), len(design)
    if min_inliers is None:
        min_inliers = _default_min_inliers(n_design)

    # * 1. Enough markers on both sides, not too many in view.
    if n_design < 3:
        return _failed_fit(f"the design has only {n_design} markers (need at least 3)", n_sensed, n_design)
    if n_sensed < 3:
        return _failed_fit(f"only {n_sensed} markers in view (need at least 3)", n_sensed, n_design)
    if n_sensed > max_sensed:
        return _failed_fit(f"too many markers in view ({n_sensed}, at most {max_sensed}); "
                           f"disable other assets or use the per-bar asset", n_sensed, n_design)

    # * 2-3. Distance tables and the start triangles of the design.
    sensed_dist = _pairwise_distances(sensed)
    design_dist = _pairwise_distances(design)
    triangles = _design_triangles(design, min_triangle_height_m)
    if not triangles:
        return _failed_fit(f"the design markers are collinear (no three of them form a triangle "
                           f"{min_triangle_height_m * 1000:.0f} mm high), so the bar's roll cannot be measured",
                           n_sensed, n_design)

    # * 4-5. Every sensed triple shaped like a design triangle is one guess. Ordered triples on purpose:
    #   M1/M2-swapped and mirrored guesses are tried too, and lose on the full-set score.
    guesses = {}
    n_hypotheses = 0
    for a, b, c in triangles:
        like_ab = np.abs(sensed_dist - design_dist[a, b]) <= side_tol_m
        like_bc = np.abs(sensed_dist - design_dist[b, c]) <= side_tol_m
        like_ac = np.abs(sensed_dist - design_dist[a, c]) <= side_tol_m
        # hits[i, j, k]: i-j looks like a-b, j-k like b-c, i-k like a-c.
        hits = like_ab[:, :, None] & like_bc[None, :, :] & like_ac[:, None, :]
        for i, j, k in zip(*np.nonzero(hits)):
            n_hypotheses += 1
            T = kabsch(design[[a, b, c]], sensed[[i, j, k]])
            pairs = _assign(sensed, transform_points(T, design), tol_m)
            if len(pairs) >= 3:
                guesses.setdefault(tuple(sorted(pairs.items())), pairs)
    if not guesses:
        return _failed_fit("no three sensed markers match any design triangle "
                           "(wrong bar, or wrong unit / axis convention?)", n_sensed, n_design, n_hypotheses)

    # * 6. Refine every distinct guess, so equal assignments end up equal; best first.
    fits = {}
    for pairs in guesses.values():
        T, pairs, residuals = _refine(sensed, design, pairs, tol_m)
        fits.setdefault(tuple(pairs.items()), (T, pairs, residuals))
    ranked = sorted(fits.values(), key=lambda fit: (-len(fit[1]), _rms(fit[2])))
    T, pairs, residuals = ranked[0]

    # * 7. Ambiguity: the best assignment that DISAGREES with the winner.
    second = next((fit for fit in ranked[1:] if _conflicts(pairs, fit[1])), None)
    second_best = None
    ambiguous = False
    if second is not None:
        second_best = {"n_matched": len(second[1]), "rms_m": _rms(second[2]),
                       "max_residual_m": max(second[2].values()), "correspondences": dict(second[1])}
        ambiguous = (second_best["n_matched"] == len(pairs) and second_best["max_residual_m"] <= tol_m)

    # * 8. Verdict.
    n_matched = len(pairs)
    if n_matched < min_inliers:
        reason = f"only {n_matched} of {n_design} design markers matched (need {min_inliers})"
    elif ambiguous:
        reason = "two different marker assignments fit equally well (symmetric layout?)"
    else:
        reason = ""
    return BarFit(ok=not reason, reason=reason, T_world_from_bar=T, correspondences=pairs,
                  sensed_ids=list(range(n_sensed)), residuals_m=residuals, rms_m=_rms(residuals),
                  max_residual_m=max(residuals.values()), n_matched=n_matched, n_design=n_design,
                  unmatched_sensed=[s for s in range(n_sensed) if s not in pairs],
                  missing_design=sorted(set(range(n_design)) - set(pairs.values())),
                  ambiguous=ambiguous, second_best=second_best, n_hypotheses=n_hypotheses)


def fit_near_hint(sensed: np.ndarray, design: np.ndarray, T_hint: np.ndarray, *, gate_m: float,
                  tol_m: float, min_inliers: int | None) -> BarFit | None:
    """The quick fit while following a bar live: match around its last pose, no full search (BMF:784-833).

    The design markers are placed at ``T_hint``; each sensed marker within ``gate_m`` of one is taken as that marker
    (closest pairs first, one to one), then the pose is refitted and the markers re-paired within ``tol_m``, until
    the pairs stop changing.

    ! Only a fit that matches EVERY design marker is kept: with a marker missing, following a pose could stay locked
      on a wrong, partly matching assignment (B20 has one that matches 6 of its 8 balls), so the caller runs the
      full search with its ambiguity check instead.

    Args:
        sensed: (N, 3) sensed points.
        design: (M, 3) design points in the bar frame.
        T_hint: 4x4 bar pose to start from (the last good fit).
        gate_m: How far a sensed marker may be from its design marker at ``T_hint``.
        tol_m: Match distance of the refined fit; every residual must stay below it.
        min_inliers: Matched markers needed; None = as in `match_and_fit`.

    Returns:
        BarFit | None: An ok fit marked ``tracked``; None when a design marker is not matched (hidden, or the bar
        moved too far from ``T_hint``) or a residual is above ``tol_m``, so the caller runs the full search.
    """
    n_sensed, n_design = len(sensed), len(design)
    if min_inliers is None:
        min_inliers = _default_min_inliers(n_design)
    if n_sensed < 3 or n_design < 3:
        return None
    pairs = _assign(sensed, transform_points(T_hint, design), gate_m)
    if len(pairs) < 3:
        return None
    # Re-pair and refit until the pairs stop changing (a few rounds at most).
    for _ in range(4):
        T, refined, residuals = _refine(sensed, design, pairs, tol_m)
        if refined == pairs:
            break
        pairs = refined
    if len(pairs) < max(min_inliers, n_design) or max(residuals.values()) > tol_m:
        return None
    return BarFit(ok=True, reason="", T_world_from_bar=T, correspondences=pairs,
                  sensed_ids=list(range(n_sensed)), residuals_m=residuals, rms_m=_rms(residuals),
                  max_residual_m=max(residuals.values()), n_matched=len(pairs), n_design=n_design,
                  unmatched_sensed=[s for s in range(n_sensed) if s not in pairs],
                  missing_design=sorted(set(range(n_design)) - set(pairs.values())), tracked=True)
