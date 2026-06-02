#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert an Omni-LIVO exported dataset into an undistorted pinhole dataset.

Input dataset layout:
  dataset/
    images/
    transforms.json

Output dataset layout:
  output/
    images/
    transforms.json

The input transforms.json must contain:
  meta.source_intrinsics = {xi, fu, fv, pu, pv, distortion_d0_d1_d2_d3}

This script keeps the original camera poses, but rewrites the images and
top-level intrinsics to a pinhole camera with zero distortion.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np


def load_json(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def omni_project(points_cam: np.ndarray, intr: Dict) -> Tuple[np.ndarray, np.ndarray]:
    x = points_cam[:, 0]
    y = points_cam[:, 1]
    z = points_cam[:, 2]
    d = np.linalg.norm(points_cam, axis=1)
    denom = z + intr["xi"] * d
    valid = denom > 1e-8

    xn = np.zeros_like(x)
    yn = np.zeros_like(y)
    xn[valid] = x[valid] / denom[valid]
    yn[valid] = y[valid] / denom[valid]

    r2 = xn * xn + yn * yn
    k1, k2, p1, p2 = intr["distortion"]
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    x_tan = 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y_tan = p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn

    xd = xn * radial + x_tan
    yd = yn * radial + y_tan

    u = intr["fu"] * xd + intr["pu"]
    v = intr["fv"] * yd + intr["pv"]
    return np.stack([u, v], axis=1).astype(np.float32), valid


def build_pinhole_remap(
    source_intr: Dict,
    out_width: int,
    out_height: int,
    out_fx: float,
    out_fy: float,
    out_cx: float,
    out_cy: float,
) -> Tuple[np.ndarray, np.ndarray]:
    grid_x, grid_y = np.meshgrid(
        np.arange(out_width, dtype=np.float32),
        np.arange(out_height, dtype=np.float32),
        indexing="xy",
    )
    x = (grid_x - out_cx) / out_fx
    y = (grid_y - out_cy) / out_fy
    dirs = np.stack([x, y, np.ones_like(x)], axis=-1).reshape(-1, 3)
    uv, valid = omni_project(dirs, source_intr)

    map_x = uv[:, 0].reshape(out_height, out_width)
    map_y = uv[:, 1].reshape(out_height, out_width)

    valid = valid.reshape(out_height, out_width)
    map_x[~valid] = -1.0
    map_y[~valid] = -1.0
    return map_x, map_y


def make_pinhole_intrinsics(
    src_width: int,
    src_height: int,
    src_fu: float,
    src_fv: float,
    scale: float,
    fov_deg: float,
) -> Dict:
    out_width = int(round(src_width * scale))
    out_height = int(round(src_height * scale))
    out_cx = out_width / 2.0
    out_cy = out_height / 2.0

    if fov_deg > 0.0:
        fov_rad = np.deg2rad(fov_deg)
        out_fx = (out_width / 2.0) / np.tan(fov_rad / 2.0)
        out_fy = (out_height / 2.0) / np.tan(fov_rad / 2.0)
    else:
        out_fx = src_fu * scale
        out_fy = src_fv * scale

    return {
        "width": out_width,
        "height": out_height,
        "fl_x": float(out_fx),
        "fl_y": float(out_fy),
        "cx": float(out_cx),
        "cy": float(out_cy),
    }


def convert_dataset(args: argparse.Namespace) -> None:
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    transforms_path = input_dir / "transforms.json"
    data = load_json(transforms_path)

    def _build_source_intrinsics(camera_meta: Dict) -> Dict:
        src = camera_meta["source_intrinsics"]
        return {
            "xi": float(src["xi"]),
            "fu": float(src["fu"]),
            "fv": float(src["fv"]),
            "pu": float(src["pu"]),
            "pv": float(src["pv"]),
            "distortion": [float(v) for v in src["distortion_d0_d1_d2_d3"]],
        }

    cameras_in = data.get("cameras")
    if cameras_in:
        camera_entries = {
            int(camera_id): camera_meta for camera_id, camera_meta in cameras_in.items()
        }
    else:
        legacy_camera_id = int(data.get("camera_id", 0))
        legacy_meta = dict(data)
        legacy_meta["source_intrinsics"] = data["meta"]["source_intrinsics"]
        camera_entries = {legacy_camera_id: legacy_meta}

    camera_models = {}
    for camera_id, camera_meta in camera_entries.items():
        source_intr = _build_source_intrinsics(camera_meta)
        src_width = int(camera_meta.get("w", camera_meta.get("width", data["w"])))
        src_height = int(camera_meta.get("h", camera_meta.get("height", data["h"])))
        pinhole_intr = make_pinhole_intrinsics(
            src_width=src_width,
            src_height=src_height,
            src_fu=source_intr["fu"],
            src_fv=source_intr["fv"],
            scale=args.scale,
            fov_deg=args.fov_deg,
        )
        map_x, map_y = build_pinhole_remap(
            source_intr=source_intr,
            out_width=pinhole_intr["width"],
            out_height=pinhole_intr["height"],
            out_fx=pinhole_intr["fl_x"],
            out_fy=pinhole_intr["fl_y"],
            out_cx=pinhole_intr["cx"],
            out_cy=pinhole_intr["cy"],
        )
        camera_models[camera_id] = {
            "source_intr": source_intr,
            "pinhole_intr": pinhole_intr,
            "map_x": map_x,
            "map_y": map_y,
            "camera_meta": camera_meta,
        }

    out_images_dir = output_dir / "images"
    out_images_dir.mkdir(parents=True, exist_ok=True)

    frames_out = []
    frames = data["frames"]
    if args.max_frames > 0:
        frames = frames[: args.max_frames]

    for idx, frame in enumerate(frames):
        camera_id = int(frame.get("camera_id", data.get("camera_id", 0)))
        if camera_id not in camera_models:
            raise ValueError(f"Frame references unknown camera_id={camera_id}")
        src_path = input_dir / frame["file_path"]
        image = cv2.imread(str(src_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Failed to read image: {src_path}")

        camera_model = camera_models[camera_id]
        undist = cv2.remap(
            image,
            camera_model["map_x"],
            camera_model["map_y"],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        out_name = Path(frame["file_path"]).stem + f"_cam{camera_id}.png"
        out_rel = Path("images") / out_name
        out_path = output_dir / out_rel
        ok = cv2.imwrite(str(out_path), undist)
        if not ok:
            raise ValueError(f"Failed to write image: {out_path}")

        frame_out = dict(frame)
        frame_out["file_path"] = str(out_rel)
        frame_out["image_path"] = str(out_path.resolve())
        frames_out.append(frame_out)

        if (idx + 1) % 20 == 0 or idx + 1 == len(frames):
            print(f"[{idx + 1}/{len(frames)}] converted")

    out_data = dict(data)
    first_camera_id = int(frames_out[0].get("camera_id", data.get("camera_id", 0)))
    first_pinhole = camera_models[first_camera_id]["pinhole_intr"]
    out_data["camera_model"] = "PINHOLE"
    out_data["width"] = first_pinhole["width"]
    out_data["height"] = first_pinhole["height"]
    out_data["w"] = first_pinhole["width"]
    out_data["h"] = first_pinhole["height"]
    out_data["fl_x"] = first_pinhole["fl_x"]
    out_data["fl_y"] = first_pinhole["fl_y"]
    out_data["cx"] = first_pinhole["cx"]
    out_data["cy"] = first_pinhole["cy"]
    out_data["k1"] = 0.0
    out_data["k2"] = 0.0
    out_data["k3"] = 0.0
    out_data["k4"] = 0.0
    out_data["frames"] = frames_out
    if data.get("cameras"):
        out_cameras = {}
        for camera_id, camera_model in camera_models.items():
            pinhole_intr = camera_model["pinhole_intr"]
            camera_meta = dict(camera_model["camera_meta"])
            camera_meta["camera_model"] = "PINHOLE"
            camera_meta["width"] = pinhole_intr["width"]
            camera_meta["height"] = pinhole_intr["height"]
            camera_meta["w"] = pinhole_intr["width"]
            camera_meta["h"] = pinhole_intr["height"]
            camera_meta["fl_x"] = pinhole_intr["fl_x"]
            camera_meta["fl_y"] = pinhole_intr["fl_y"]
            camera_meta["cx"] = pinhole_intr["cx"]
            camera_meta["cy"] = pinhole_intr["cy"]
            camera_meta["k1"] = 0.0
            camera_meta["k2"] = 0.0
            camera_meta["k3"] = 0.0
            camera_meta["k4"] = 0.0
            out_cameras[str(camera_id)] = camera_meta
        out_data["cameras"] = out_cameras
    out_data.setdefault("meta", {})
    out_data["meta"]["converted_from"] = str(input_dir.resolve())
    out_data["meta"]["conversion"] = {
        "type": "omni_to_pinhole",
        "fov_deg": args.fov_deg,
        "scale": args.scale,
    }

    save_json(output_dir / "transforms.json", out_data)
    print(f"Saved pinhole dataset to {output_dir}")


def compare_fovs(args: argparse.Namespace) -> None:
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    transforms_path = input_dir / "transforms.json"
    data = load_json(transforms_path)
    src = data["meta"]["source_intrinsics"]
    src_width = int(data["w"])
    src_height = int(data["h"])
    source_intr = {
        "xi": float(src["xi"]),
        "fu": float(src["fu"]),
        "fv": float(src["fv"]),
        "pu": float(src["pu"]),
        "pv": float(src["pv"]),
        "distortion": [float(v) for v in src["distortion_d0_d1_d2_d3"]],
    }

    frames = data["frames"]
    if not frames:
        raise ValueError("No frames found in input dataset.")

    sample_indices = [0]
    if len(frames) > 1:
        sample_indices.append(len(frames) // 2)
    sample_indices = sample_indices[: max(1, args.max_frames)]

    fov_list = [float(v) for v in args.fovs.split(",") if v.strip()]
    for sample_rank, frame_idx in enumerate(sample_indices):
        frame = frames[frame_idx]
        src_path = input_dir / frame["file_path"]
        image = cv2.imread(str(src_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Failed to read image: {src_path}")

        panels: List[np.ndarray] = []
        original = image.copy()
        cv2.putText(
            original,
            "original",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        panels.append(original)

        for fov_deg in fov_list:
            pinhole_intr = make_pinhole_intrinsics(
                src_width=src_width,
                src_height=src_height,
                src_fu=source_intr["fu"],
                src_fv=source_intr["fv"],
                scale=args.scale,
                fov_deg=fov_deg,
            )
            map_x, map_y = build_pinhole_remap(
                source_intr=source_intr,
                out_width=pinhole_intr["width"],
                out_height=pinhole_intr["height"],
                out_fx=pinhole_intr["fl_x"],
                out_fy=pinhole_intr["fl_y"],
                out_cx=pinhole_intr["cx"],
                out_cy=pinhole_intr["cy"],
            )
            undist = cv2.remap(
                image,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
            show = undist.copy()
            cv2.putText(
                show,
                f"fov={fov_deg:.0f}",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            panels.append(show)

        target_h = min(panel.shape[0] for panel in panels)
        resized = []
        for panel in panels:
            if panel.shape[0] != target_h:
                new_w = int(round(panel.shape[1] * target_h / panel.shape[0]))
                panel = cv2.resize(panel, (new_w, target_h), interpolation=cv2.INTER_LINEAR)
            resized.append(panel)

        canvas = np.concatenate(resized, axis=1)
        out_path = output_dir / f"compare_fov_frame{sample_rank:02d}_{Path(frame['file_path']).stem}.png"
        cv2.imwrite(str(out_path), canvas)
        print(f"Saved FOV comparison to {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert Omni-LIVO exported dataset to undistorted pinhole dataset.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    convert_parser = subparsers.add_parser("convert", help="Convert dataset to pinhole")
    convert_parser.add_argument("--input-dir", required=True, help="Input dataset directory containing images/ and transforms.json")
    convert_parser.add_argument("--output-dir", required=True, help="Output directory for pinhole dataset")
    convert_parser.add_argument("--fov-deg", type=float, default=120.0, help="Target pinhole horizontal/vertical FOV in degrees")
    convert_parser.add_argument("--scale", type=float, default=1.0, help="Output image scale")
    convert_parser.add_argument("--max-frames", type=int, default=0, help="Optional limit on number of frames to convert")
    convert_parser.set_defaults(func=convert_dataset)

    compare_parser = subparsers.add_parser("compare-fov", help="Generate side-by-side FOV comparison images")
    compare_parser.add_argument("--input-dir", required=True, help="Input dataset directory containing images/ and transforms.json")
    compare_parser.add_argument("--output-dir", required=True, help="Directory for FOV comparison images")
    compare_parser.add_argument("--fovs", default="90,100,110,120", help="Comma-separated FOV list")
    compare_parser.add_argument("--scale", type=float, default=0.5, help="Output image scale")
    compare_parser.add_argument("--max-frames", type=int, default=2, help="How many sample frames to compare")
    compare_parser.set_defaults(func=compare_fovs)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
