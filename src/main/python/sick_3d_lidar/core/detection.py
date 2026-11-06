"""Fixed-sensor foreground clustering and best-observation passage export."""

from dataclasses import dataclass, replace
from itertools import product
import logging
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from sick_3d_lidar.core.compact import Segment, write_pcd


logger = logging.getLogger(__name__)
PointArray = NDArray[np.float32]
NEIGHBORS = tuple(product((-1, 0, 1), repeat=3))


def voxelize(points: PointArray, size: float) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
    if not len(points):
        return np.empty((0, 3), dtype=np.int64), np.empty(0, dtype=np.int64)
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            scaled = np.floor(points[:, :3].astype(np.float64) / size)
    except FloatingPointError as error:
        raise ValueError("Voxel size is too small for the point coordinates.") from error
    if np.any(np.abs(scaled) >= 2**62):
        raise ValueError("Voxel indices exceed the supported integer range.")
    cells = scaled.astype(np.int64)
    origin = cells.min(axis=0)
    spans = cells.max(axis=0) - origin + 1
    if math.prod(int(value) for value in spans) > np.iinfo(np.int64).max:
        raise ValueError("Voxel grid is too large; increase the voxel size or restrict the ROI.")
    shifted = cells - origin
    # Sorting scalar keys is substantially faster than sorting XYZ records.
    keys = (shifted[:, 0] * spans[1] + shifted[:, 1]) * spans[2] + shifted[:, 2]
    _, indices, inverse = np.unique(keys, return_index=True, return_inverse=True)
    return cells[indices], inverse


@dataclass(frozen=True)
class DetectionConfig:
    background_frames: int = 10
    voxel_size: float = 0.15
    change_distance: float = 0.45
    cluster_size: float = 0.6
    min_points: int = 50
    min_voxels: int = 3
    min_hits: int = 3
    track_distance: float = 2.0
    exit_seconds: float = 1.0
    min_missed_frames: int = 3
    min_range: float = 0.5
    max_range: float = 80.0
    roi: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        for value in (
            self.background_frames, self.min_points, self.min_voxels,
            self.min_hits, self.min_missed_frames,
        ):
            if not isinstance(value, int) or value < 1:
                raise ValueError("Frame, point, voxel and hit counts must be positive integers.")
        for value in (
            self.voxel_size, self.change_distance, self.cluster_size,
            self.track_distance, self.exit_seconds, self.max_range,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Detection distances and timeouts must be finite and positive.")
        if not math.isfinite(self.min_range) or not 0 <= self.min_range < self.max_range:
            raise ValueError("Range limits require 0 <= min_range < max_range.")
        if self.change_distance < math.sqrt(3) * self.voxel_size:
            raise ValueError("change_distance must be at least sqrt(3) * voxel_size.")
        if self.roi is not None:
            if len(self.roi) != 6 or not all(math.isfinite(v) for v in self.roi):
                raise ValueError("ROI requires six finite XYZ limits.")
            if any(self.roi[i] >= self.roi[i + 1] for i in (0, 2, 4)):
                raise ValueError("ROI minimum must be smaller than maximum on each axis.")


class FrameAssembler:
    """Only complete, consistent sensor frames may update the detector."""

    def __init__(self) -> None:
        self.pending: dict[int, Segment] = {}
        self.last: Segment | None = None
        self.signature: tuple[int, ...] | None = None
        self.completed_frame: int | None = None

    def add(self, segment: Segment) -> Segment | None:
        signature = (
            segment.serial, segment.rows, segment.echoes, segment.segments_per_frame,
            segment.frame_columns, segment.interlace_steps, segment.scan_configuration,
        )
        if self.signature is not None and signature != self.signature:
            raise ValueError("Sensor/scan configuration changed; restart background training.")
        self.signature = signature
        if min(segment.rows, segment.columns, segment.echoes) <= 0:
            raise ValueError("Detection requires nonempty scan dimensions.")
        if self.last is not None:
            if segment.sequence <= self.last.sequence:
                raise ValueError("Non-increasing telegram sequence; restart after sensor reset.")
            if segment.sequence != self.last.sequence + 1:
                logger.warning(
                    "Telegram sequence gap: %s -> %s",
                    self.last.sequence, segment.sequence,
                )
            if segment.frame < self.last.frame or segment.timestamp_us < self.last.timestamp_us:
                raise ValueError("Sensor frame or timestamp moved backwards.")
            if segment.frame != self.last.frame:
                if segment.timestamp_us <= self.last.timestamp_us:
                    raise ValueError("New frames require increasing sensor timestamps.")
                self.discard_incomplete()
        if segment.frame == self.completed_frame:
            raise ValueError("Duplicate segment for an already completed frame.")
        if self.pending:
            first = next(iter(self.pending.values()))
            if (
                segment.timestamp_us != first.timestamp_us
                or segment.interlace_index != first.interlace_index
            ):
                raise ValueError("Segments in one frame disagree on timestamp or interlace.")
        if segment.index in self.pending:
            raise ValueError("Duplicate segment index.")
        self.last = segment
        self.pending[segment.index] = segment
        if len(self.pending) != segment.segments_per_frame:
            return None
        ordered = [self.pending[i] for i in range(segment.segments_per_frame)]
        if sum(item.columns for item in ordered) != segment.frame_columns:
            raise ValueError("Complete frame columns do not match frame metadata.")
        frame = replace(
            ordered[0], columns=segment.frame_columns,
            points=b"".join(item.points for item in ordered),
            zero_distances=sum(item.zero_distances for item in ordered),
        )
        self.pending.clear()
        self.completed_frame = frame.frame
        return frame

    def discard_incomplete(self) -> None:
        if self.pending:
            first = next(iter(self.pending.values()))
            logger.warning(
                "Discarding incomplete frame %s (%s/%s segments); "
                "not counted as an empty detection frame",
                first.frame, len(self.pending), first.segments_per_frame,
            )
            self.pending.clear()


@dataclass
class Track:
    identifier: int
    centroid: NDArray[np.float64]
    velocity: NDArray[np.float64]
    first_us: int
    last_us: int
    best_frame: Segment
    best_points: PointArray
    hits: int = 1
    misses: int = 0


class ObjectDetector:
    def __init__(self, config: DetectionConfig, output_dir: Path):
        self.config = config
        self.output_dir = output_dir
        output_dir.mkdir(parents=True, exist_ok=False)
        self.background_parts: list[NDArray[np.int64]] = []
        self.background_counts: dict[int, int] = {}
        self.background: cKDTree | None = None
        self.tracks: dict[int, Track] = {}
        self.next_id = 1
        self.saved = 0
        self.frames = 0

    def _filtered_points(self, frame: Segment) -> PointArray:
        points = np.frombuffer(frame.points, dtype="<f4").reshape(-1, 4)
        xyz = points[:, :3]
        ranges = np.linalg.norm(xyz.astype(np.float64), axis=1)
        selected = (ranges >= self.config.min_range) & (
            ranges <= self.config.max_range
        )
        if self.config.roi is not None:
            limits = np.asarray(self.config.roi).reshape(3, 2)
            selected &= np.all((xyz >= limits[:, 0]) & (xyz <= limits[:, 1]), axis=1)
        return points[selected]

    def _foreground(self, frame: Segment, points: PointArray) -> PointArray | None:
        cells, inverse = voxelize(points, self.config.voxel_size)
        if self.background is None:
            phase = frame.interlace_index
            count = self.background_counts.get(phase, 0)
            if count < self.config.background_frames:
                self.background_parts.append(cells)
                self.background_counts[phase] = count + 1
            phases = max(1, frame.interlace_steps)
            received = sum(self.background_counts.values())
            required = phases * self.config.background_frames
            logger.info("Empty-scene background: %s/%s frames", received, required)
            if len(self.background_counts) == phases and received == required:
                background = np.unique(np.concatenate(self.background_parts), axis=0)
                if not len(background):
                    raise ValueError("No background points in the configured range/ROI.")
                self.background = cKDTree((background + 0.5) * self.config.voxel_size)
                self.background_parts.clear()
                logger.info("Background ready; monitoring new surfaces")
            return None
        distances, _ = self.background.query(
            (cells + 0.5) * self.config.voxel_size,
            distance_upper_bound=self.config.change_distance,
        )
        return points[(distances > self.config.change_distance)[inverse]]

    def _clusters(self, points: PointArray) -> list[PointArray]:
        if len(points) < self.config.min_points:
            return []
        cells, inverse = voxelize(points, self.config.cluster_size)
        lookup = {tuple(cell): index for index, cell in enumerate(cells.tolist())}
        labels = np.full(len(cells), -1, dtype=np.int64)
        components: list[int] = []
        while lookup:
            cell, index = lookup.popitem()
            label = len(components)
            labels[index] = label
            stack = [cell]
            size = 0
            while stack:
                current = stack.pop()
                size += 1
                for dx, dy, dz in NEIGHBORS:
                    neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                    neighbor_index = lookup.pop(neighbor, None)
                    if neighbor_index is not None:
                        labels[neighbor_index] = label
                        stack.append(neighbor)
            components.append(size)
        point_labels = labels[inverse]
        sizes = np.bincount(point_labels, minlength=len(components))
        return [
            points[point_labels == index]
            for index, voxels in enumerate(components)
            if voxels >= self.config.min_voxels and sizes[index] >= self.config.min_points
        ]

    def process(self, frame: Segment) -> None:
        self.frames += 1
        foreground = self._foreground(frame, self._filtered_points(frame))
        if foreground is None:
            return
        clusters = self._clusters(foreground)
        centers = [np.mean(points[:, :3], axis=0, dtype=np.float64) for points in clusters]
        candidates = []
        for identifier, track in self.tracks.items():
            dt = (frame.timestamp_us - track.last_us) / 1e6
            predicted = track.centroid + track.velocity * min(dt, self.config.exit_seconds)
            for index, center in enumerate(centers):
                distance = float(np.linalg.norm(center - predicted))
                if distance <= self.config.track_distance:
                    candidates.append((distance, identifier, index))
        matched_tracks: set[int] = set()
        matched_clusters: set[int] = set()
        for _, identifier, index in sorted(candidates):
            if identifier in matched_tracks or index in matched_clusters:
                continue
            track = self.tracks[identifier]
            dt = (frame.timestamp_us - track.last_us) / 1e6
            if dt > 0:
                track.velocity = (centers[index] - track.centroid) / dt
            track.centroid = centers[index]
            track.last_us = frame.timestamp_us
            track.hits += 1
            track.misses = 0
            if len(clusters[index]) > len(track.best_points):
                track.best_points = clusters[index]
                track.best_frame = replace(frame, points=b"")
            matched_tracks.add(identifier)
            matched_clusters.add(index)
        for identifier, track in list(self.tracks.items()):
            if identifier in matched_tracks:
                continue
            track.misses += 1
            if (
                track.misses >= self.config.min_missed_frames
                and (frame.timestamp_us - track.last_us) / 1e6 >= self.config.exit_seconds
            ):
                self._save(track, "departed")
                del self.tracks[identifier]
        for index, cluster in enumerate(clusters):
            if index not in matched_clusters:
                if len(self.tracks) >= 1000:
                    raise RuntimeError("Too many tracks; restrict ROI or raise cluster thresholds.")
                self.tracks[self.next_id] = Track(
                    self.next_id, centers[index], np.zeros(3), frame.timestamp_us,
                    frame.timestamp_us, replace(frame, points=b""), cluster,
                )
                logger.info("New candidate object %s (%s points)", self.next_id, len(cluster))
                self.next_id += 1
        if self.frames % 20 == 0:
            logger.info(
                "Frame %s: %s changed points, %s clusters, %s tracks, %s PCD saved",
                frame.frame, len(foreground), len(clusters), len(self.tracks), self.saved,
            )

    def _save(self, track: Track, reason: str) -> None:
        if track.hits < self.config.min_hits:
            logger.info("Discarding transient candidate %s (%s hits)", track.identifier, track.hits)
            return
        frame = track.best_frame
        partial = reason != "departed"
        suffix = "_partial" if partial else ""
        path = self.output_dir / (
            f"object_{track.identifier:06d}_sensor_{frame.serial}"
            f"_frame_{frame.frame}{suffix}.pcd"
        )
        write_pcd(path, track.best_points.astype("<f4", copy=False).tobytes(), (
            f"object_id {track.identifier}",
            f"end_reason {reason}",
            f"complete_passage {str(not partial).lower()}",
            f"first_seen_us {track.first_us}",
            f"last_seen_us {track.last_us}",
            f"best_frame_timestamp_us {frame.timestamp_us}",
            f"interlace {frame.interlace_index}/{frame.interlace_steps}",
            f"observations {track.hits}",
            "foreground_only true; not a vehicle classification or full-body model",
        ))
        self.saved += 1
        logger.info("Saved object %s: %s points -> %s", track.identifier, len(track.best_points), path)

    def finish(self, reason: str) -> None:
        for identifier, track in list(self.tracks.items()):
            self._save(track, reason)
            del self.tracks[identifier]
        if self.background is None:
            raise ValueError("Stopped before all interlace phases completed background training.")
