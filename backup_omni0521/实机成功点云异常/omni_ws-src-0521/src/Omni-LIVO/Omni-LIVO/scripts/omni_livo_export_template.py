#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Omni-LIVO export template and projection checker.

This script provides a practical Python skeleton for:
1. Building an image list.
2. Converting Omni-LIVO poses to world-to-camera transforms.
3. Exporting omnidirectional fisheye intrinsics and distortion parameters.
4. Running a simple projection sanity check.

Assumptions:
- Pose file format matches Log/result/frame_pose.txt:
  timestamp tx ty tz qx qy qz qw
- The pose is the IMU pose in world coordinates.
- Extrinsics are read from Omni-LIVO config:
  extrin_calib/extrinsic_R, extrin_calib/extrinsic_T, extrin_calib/cameras[*].Rcl, Pcl
- camera_intrinsics.yaml uses the OpenCV omnidir-style parameter layout:
  xi, fu, fv, pu, pv, d0, d1, d2, d3

Example:
  python omni_livo_export_template.py export \
      --pose-file ../Log/result/frame_pose.txt \
      --config ../config/mid360_multi_cam.yaml \
      --intrinsics ../config/camera_intrinsics.yaml \
      --image-dir /path/to/images \
      --camera-id 0 \
      --output export_cam0.json

  python omni_livo_export_template.py check \
      --export-json export_cam0.json \
      --frame-index 0 \
      --world-point 1.0 0.0 0.0 \
      --overlay-output check_cam0.png
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml


@dataclass
class PoseRecord:
    timestamp: float
    t_wi: List[float]
    q_wi_xyzw: List[float]


@dataclass
class CameraIntrinsics:
    model: str
    width: int
    height: int
    xi: float
    fu: float
    fv: float
    pu: float
    pv: float
    distortion: List[float]


@dataclass
class CameraExtrinsics:
    camera_id: int
    topic: str
    r_cl: List[List[float]]
    p_cl: List[float]
    r_ci: List[List[float]]
    p_ci: List[float]


@dataclass
class ExportFrame:
    image_path: str
    image_name: str
    timestamp: float
    pose_source_timestamp: float
    camera_id: int
    t_wc: List[float]
    q_wc_wxyz: List[float]
    w2c: List[List[float]]


def quaternion_xyzw_to_rotation_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = q_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Zero-norm quaternion in pose file.")
    qx, qy, qz, qw = (qx / norm, qy / norm, qz / norm, qw / norm)

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion_wxyz(r: np.ndarray) -> List[float]:
    trace = np.trace(r)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    return [float(qw), float(qx), float(qy), float(qz)]


def make_transform(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r
    transform[:3, 3] = t.reshape(3)
    return transform


def parse_pose_file(path: Path) -> List[PoseRecord]:
    poses: List[PoseRecord] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(f"Invalid pose line {line_no}: expected 8 columns, got {len(parts)}")
            vals = [float(x) for x in parts]
            poses.append(
                PoseRecord(
                    timestamp=vals[0],
                    t_wi=vals[1:4],
                    q_wi_xyzw=vals[4:8],
                )
            )
    if not poses:
        raise ValueError(f"No pose records found in {path}")
    return poses


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def compute_camera_extrinsics(config: dict, camera_id: int) -> CameraExtrinsics:
    extrin = config["extrin_calib"]
    r_il = np.array(extrin["extrinsic_R"], dtype=np.float64).reshape(3, 3)
    t_il = np.array(extrin["extrinsic_T"], dtype=np.float64).reshape(3)

    r_li = r_il.T
    p_li = -r_il.T @ t_il

    cameras = extrin["cameras"]
    camera_node = next((cam for cam in cameras if int(cam["cam_id"]) == camera_id), None)
    if camera_node is None:
        raise ValueError(f"Camera id {camera_id} not found in extrin_calib/cameras")

    r_cl = np.array(camera_node["Rcl"], dtype=np.float64).reshape(3, 3)
    p_cl = np.array(camera_node["Pcl"], dtype=np.float64).reshape(3)

    r_ci = r_cl @ r_li
    p_ci = r_cl @ p_li + p_cl

    return CameraExtrinsics(
        camera_id=camera_id,
        topic=str(camera_node.get("img_topic", "")),
        r_cl=r_cl.tolist(),
        p_cl=p_cl.tolist(),
        r_ci=r_ci.tolist(),
        p_ci=p_ci.tolist(),
    )


def load_intrinsics(intrinsics_yaml: dict, camera_id: int) -> CameraIntrinsics:
    key = f"cam_{camera_id}"
    cam = intrinsics_yaml.get(key)
    if cam is None:
        raise ValueError(f"Missing intrinsics entry: {key}")
    return CameraIntrinsics(
        model=str(cam["cam_model"]),
        width=int(cam["cam_width"]),
        height=int(cam["cam_height"]),
        xi=float(cam["cam_xi"]),
        fu=float(cam["cam_fu"]),
        fv=float(cam["cam_fv"]),
        pu=float(cam["cam_pu"]),
        pv=float(cam["cam_pv"]),
        distortion=[
            float(cam["cam_d0"]),
            float(cam["cam_d1"]),
            float(cam["cam_d2"]),
            float(cam["cam_d3"]),
        ],
    )


def parse_timestamp_from_stem(path: Path) -> float:
    try:
        return float(path.stem)
    except ValueError as exc:
        raise ValueError(
            f"Cannot parse timestamp from image filename '{path.name}'. "
            "Expected filenames like 1752315292.197933.png"
        ) from exc


def collect_images(image_dir: Path, image_glob: str) -> List[Tuple[Path, float]]:
    items = sorted(image_dir.glob(image_glob))
    if not items:
        raise ValueError(f"No images found under {image_dir} with pattern '{image_glob}'")
    image_items = [(path, parse_timestamp_from_stem(path)) for path in items if path.is_file()]
    if not image_items:
        raise ValueError(f"No valid image files found in {image_dir}")
    return image_items


def find_nearest_pose(poses: Sequence[PoseRecord], timestamp: float) -> PoseRecord:
    return min(poses, key=lambda pose: abs(pose.timestamp - timestamp))


def build_world_to_camera(
    pose: PoseRecord,
    camera_extrinsics: CameraExtrinsics,
) -> Tuple[np.ndarray, np.ndarray]:
    r_wi = quaternion_xyzw_to_rotation_matrix(pose.q_wi_xyzw)
    t_wi = np.array(pose.t_wi, dtype=np.float64)

    r_iw = r_wi.T
    r_ci = np.array(camera_extrinsics.r_ci, dtype=np.float64)
    p_ci = np.array(camera_extrinsics.p_ci, dtype=np.float64)

    r_cw = r_ci @ r_iw
    t_cw = -r_ci @ r_iw @ t_wi + p_ci
    return r_cw, t_cw


def export_frames(args: argparse.Namespace) -> None:
    pose_file = Path(args.pose_file)
    config_file = Path(args.config)
    intrinsics_file = Path(args.intrinsics)
    image_dir = Path(args.image_dir)
    output_path = Path(args.output)

    poses = parse_pose_file(pose_file)
    config = load_yaml(config_file)
    intrinsics_yaml = load_yaml(intrinsics_file)

    camera_extrinsics = compute_camera_extrinsics(config, args.camera_id)
    intrinsics = load_intrinsics(intrinsics_yaml, args.camera_id)
    images = collect_images(image_dir, args.image_glob)

    frames: List[ExportFrame] = []
    for image_path, timestamp in images:
        pose = find_nearest_pose(poses, timestamp)
        dt = abs(pose.timestamp - timestamp)
        if args.max_time_diff is not None and dt > args.max_time_diff:
            continue

        r_cw, t_cw = build_world_to_camera(pose, camera_extrinsics)
        w2c = make_transform(r_cw, t_cw)
        q_wc = rotation_matrix_to_quaternion_wxyz(r_cw)

        frames.append(
            ExportFrame(
                image_path=str(image_path.resolve()),
                image_name=image_path.name,
                timestamp=float(timestamp),
                pose_source_timestamp=float(pose.timestamp),
                camera_id=args.camera_id,
                t_wc=[float(v) for v in t_cw.tolist()],
                q_wc_wxyz=q_wc,
                w2c=w2c.tolist(),
            )
        )

    payload = {
        "meta": {
            "tool": "omni_livo_export_template.py",
            "camera_id": args.camera_id,
            "image_dir": str(image_dir.resolve()),
            "image_glob": args.image_glob,
            "pose_file": str(pose_file.resolve()),
            "config_file": str(config_file.resolve()),
            "intrinsics_file": str(intrinsics_file.resolve()),
            "matched_frames": len(frames),
            "assumption": "pose is IMU-in-world; export uses Omni-LIVO world-to-camera convention",
        },
        "camera": {
            "intrinsics": asdict(intrinsics),
            "extrinsics": asdict(camera_extrinsics),
        },
        "frames": [asdict(frame) for frame in frames],
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    print(f"Exported {len(frames)} frame(s) to {output_path}")
    print("Fields included: image list, world-to-camera pose, omnidir intrinsics, distortion.")


def project_world_point(
    world_point: Sequence[float],
    w2c: np.ndarray,
    intr: CameraIntrinsics,
) -> Tuple[np.ndarray, np.ndarray]:
    pw = np.array(world_point, dtype=np.float64).reshape(3)
    pc = w2c[:3, :3] @ pw + w2c[:3, 3]

    x, y, z = pc.tolist()
    d = math.sqrt(x * x + y * y + z * z)
    denom = z + intr.xi * d
    if denom <= 1e-12:
        raise ValueError("Point lies behind the omnidirectional camera or on an invalid projection ray.")

    xn = x / denom
    yn = y / denom
    r2 = xn * xn + yn * yn

    k1, k2, p1, p2 = intr.distortion
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    x_tan = 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y_tan = p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn

    xd = xn * radial + x_tan
    yd = yn * radial + y_tan

    u = intr.fu * xd + intr.pu
    v = intr.fv * yd + intr.pv
    return np.array([u, v], dtype=np.float64), pc


def draw_overlay(image_path: Path, uv: np.ndarray, output_path: Path) -> None:
    try:
        import cv2  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("OpenCV is not installed, cannot create overlay output.") from exc

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to read image: {image_path}")

    u = int(round(float(uv[0])))
    v = int(round(float(uv[1])))
    color = (0, 0, 255)
    cv2.drawMarker(image, (u, v), color, markerType=cv2.MARKER_CROSS, markerSize=40, thickness=2)
    cv2.putText(
        image,
        f"({uv[0]:.1f}, {uv[1]:.1f})",
        (max(10, u + 10), max(30, v - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), image)


def check_projection(args: argparse.Namespace) -> None:
    export_json = Path(args.export_json)
    with export_json.open("r", encoding="utf-8") as f:
        payload = json.load(f)

    frames = payload["frames"]
    if not frames:
        raise ValueError("Export JSON has no frames.")

    frame = frames[args.frame_index]
    intr = CameraIntrinsics(**payload["camera"]["intrinsics"])
    w2c = np.array(frame["w2c"], dtype=np.float64)

    uv, pc = project_world_point(args.world_point, w2c, intr)
    in_bounds = 0.0 <= uv[0] < intr.width and 0.0 <= uv[1] < intr.height

    print(f"image: {frame['image_name']}")
    print(f"camera_id: {frame['camera_id']}")
    print(f"world_point: {list(map(float, args.world_point))}")
    print(f"camera_point: {[float(v) for v in pc.tolist()]}")
    print(f"pixel: {[float(v) for v in uv.tolist()]}")
    print(f"in_image_bounds: {bool(in_bounds)}")

    if args.overlay_output:
        draw_overlay(Path(frame["image_path"]), uv, Path(args.overlay_output))
        print(f"overlay saved to: {args.overlay_output}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Omni-LIVO export template and projection checker")
    subparsers = parser.add_subparsers(dest="command", required=True)

    export_parser = subparsers.add_parser("export", help="Export image list, w2c poses and camera parameters")
    export_parser.add_argument("--pose-file", required=True, help="Path to frame_pose.txt")
    export_parser.add_argument("--config", required=True, help="Path to Omni-LIVO config yaml with extrinsics")
    export_parser.add_argument("--intrinsics", required=True, help="Path to camera_intrinsics.yaml")
    export_parser.add_argument("--image-dir", required=True, help="Directory containing images for one camera")
    export_parser.add_argument("--camera-id", type=int, required=True, help="Camera id to export")
    export_parser.add_argument(
        "--image-glob",
        default="*.png",
        help="Image filename pattern. Filenames must use timestamps as stem, e.g. 1752315292.123.png",
    )
    export_parser.add_argument(
        "--max-time-diff",
        type=float,
        default=None,
        help="Optional maximum allowed |image_ts - pose_ts| in seconds",
    )
    export_parser.add_argument("--output", required=True, help="Output JSON path")
    export_parser.set_defaults(func=export_frames)

    check_parser = subparsers.add_parser("check", help="Project one world point into one exported frame")
    check_parser.add_argument("--export-json", required=True, help="JSON created by the export subcommand")
    check_parser.add_argument("--frame-index", type=int, default=0, help="Frame index inside export JSON")
    check_parser.add_argument(
        "--world-point",
        type=float,
        nargs=3,
        required=True,
        metavar=("X", "Y", "Z"),
        help="3D point in world coordinates",
    )
    check_parser.add_argument(
        "--overlay-output",
        default=None,
        help="Optional output image path for a visualization overlay",
    )
    check_parser.set_defaults(func=check_projection)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
