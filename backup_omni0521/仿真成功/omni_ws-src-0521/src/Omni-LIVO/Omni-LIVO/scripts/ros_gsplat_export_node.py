#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS topic exporter for Omni-LIVO -> gsplat native fisheye-style dataset.

This node subscribes to:
- an image topic
- a pose topic (typically /aft_mapped_to_init)

It writes:
- <output_dir>/images/*.png
- <output_dir>/transforms.json

The JSON contains:
- gsplat/NeRF-like per-frame camera-to-world transforms
- top-level fisheye intrinsics and k1..k4 distortion
- optional world-to-camera matrices for convenience

Notes:
- Pose topic can be interpreted as IMU or LiDAR pose in world coordinates.
- Camera pose is recovered using Omni-LIVO extrinsics in config yaml.
- The main exported camera model is OPENCV_FISHEYE-style, using d0..d3
  as k1..k4. The original Omni-LIVO xi parameter is preserved in metadata.
"""

from __future__ import annotations

import json
import math
import threading
from collections import deque
from dataclasses import dataclass
from queue import Empty, Full, Queue
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from sensor_msgs.msg import Image

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class PoseSample:
    stamp: float
    header_stamp: float
    arrival_stamp: float
    t_wb: List[float]
    q_wb_xyzw: List[float]


@dataclass
class PendingFrame:
    image: np.ndarray
    image_path: Path
    frame: Dict


def quaternion_xyzw_to_rotation_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    qx, qy, qz, qw = q_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Zero-norm quaternion.")
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


def make_transform(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r
    transform[:3, 3] = t.reshape(3)
    return transform


def save_image_with_pillow(
    image: np.ndarray, image_path: Path, image_format: str, jpg_quality: int
) -> None:
    if image.ndim == 2:
        pil_image = PILImage.fromarray(image)
    elif image.ndim == 3 and image.shape[2] == 3:
        rgb = image[:, :, ::-1]
        pil_image = PILImage.fromarray(rgb)
    elif image.ndim == 3 and image.shape[2] == 4:
        rgba = image[:, :, [2, 1, 0, 3]]
        pil_image = PILImage.fromarray(rgba)
    else:
        raise ValueError(f"Unsupported image shape for export: {image.shape}")

    image_path.parent.mkdir(parents=True, exist_ok=True)
    fmt = image_format.upper()
    if fmt == "JPG":
        fmt = "JPEG"
    if fmt == "JPEG":
        pil_image.save(str(image_path), format=fmt, quality=jpg_quality)
    else:
        pil_image.save(str(image_path), format=fmt)


class OmniLIVOGsplatExporter:
    def __init__(self) -> None:
        rospy.init_node("omni_livo_gsplat_exporter", anonymous=False)

        default_output_dir = PACKAGE_ROOT / "GsplatExport"
        self.output_dir = Path(rospy.get_param("~output_dir", str(default_output_dir)))
        self.config_path = Path(rospy.get_param("~config_path"))
        self.intrinsics_path = Path(rospy.get_param("~intrinsics_path"))
        self.camera_id = int(rospy.get_param("~camera_id", 0))
        self.camera_ids_param = str(rospy.get_param("~camera_ids", "")).strip()
        self.pose_topic = rospy.get_param("~pose_topic", "/aft_mapped_to_init")
        self.pose_frame = self._validate_pose_frame(
            rospy.get_param("~pose_frame", "imu").strip().lower()
        )
        self.image_topic = rospy.get_param("~image_topic", "")
        self.image_format = rospy.get_param("~image_format", "png")
        self.jpg_quality = int(rospy.get_param("~jpg_quality", 95))
        self.image_prefix = rospy.get_param("~image_prefix", "frame")
        self.save_every_n = max(1, int(rospy.get_param("~save_every_n", 1)))
        self.writer_queue_size = max(8, int(rospy.get_param("~writer_queue_size", 128)))
        self.max_pose_age = float(rospy.get_param("~max_pose_age", 0.05))
        self.pose_buffer_size = max(10, int(rospy.get_param("~pose_buffer_size", 200)))
        self.sync_mode = rospy.get_param("~sync_mode", "nearest").strip().lower()
        self.pose_time_offset = float(rospy.get_param("~pose_time_offset", 0.0))
        self.image_time_offset = float(rospy.get_param("~image_time_offset", 0.0))
        self.pose_stamp_source = self._validate_stamp_source(
            rospy.get_param("~pose_stamp_source", "header").strip().lower(),
            "~pose_stamp_source",
        )
        self.image_stamp_source = self._validate_stamp_source(
            rospy.get_param("~image_stamp_source", "header").strip().lower(),
            "~image_stamp_source",
        )
        self.use_header_time_for_name = bool(rospy.get_param("~use_header_time_for_name", True))
        self.encoding = rospy.get_param("~encoding", "bgr8")
        self._last_sync_warning_key: Optional[str] = None

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.pose_buffer: Deque[PoseSample] = deque(maxlen=self.pose_buffer_size)
        self.frames: List[Dict] = []
        self.saved_count = 0
        self.received_images = 0
        self.received_images_by_camera: Dict[int, int] = {}
        self.enqueued_images = 0
        self.dropped_images = 0
        self.first_received_stamp: Optional[float] = None
        self.last_received_stamp: Optional[float] = None
        self.first_saved_stamp: Optional[float] = None
        self.last_saved_stamp: Optional[float] = None
        self.write_queue: Queue = Queue(maxsize=self.writer_queue_size)
        self.stop_writer = False
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)

        self.images_dir = self.output_dir / "images"
        self.images_dir.mkdir(parents=True, exist_ok=True)

        self.config = self._load_yaml(self.config_path)
        self.intrinsics_yaml = self._load_yaml(self.intrinsics_path)
        self.camera_ids = self._resolve_camera_ids()
        self.intrinsics_map = {
            cam_id: self._load_intrinsics(cam_id) for cam_id in self.camera_ids
        }
        self.camera_extrinsics_map = {
            cam_id: self._load_extrinsics(cam_id) for cam_id in self.camera_ids
        }
        self.received_images_by_camera = {cam_id: 0 for cam_id in self.camera_ids}

        if self.image_topic and len(self.camera_ids) > 1:
            raise ValueError("Please do not set ~image_topic when exporting multiple cameras.")
        if not self.image_topic:
            if len(self.camera_ids) == 1:
                self.image_topic = self.camera_extrinsics_map[self.camera_ids[0]]["topic"]
            else:
                self.image_topic = ""
        if len(self.camera_ids) == 1 and not self.image_topic:
            raise ValueError("image_topic is empty. Please set ~image_topic or provide img_topic in config.")

        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.pose_callback, queue_size=200)
        self.image_subs = []
        if len(self.camera_ids) == 1:
            cam_id = self.camera_ids[0]
            self.image_subs.append(
                rospy.Subscriber(
                    self.image_topic,
                    Image,
                    lambda msg, cam_id=cam_id: self.image_callback(cam_id, msg),
                    queue_size=20,
                )
            )
        else:
            for cam_id in self.camera_ids:
                topic = self.camera_extrinsics_map[cam_id]["topic"]
                if not topic:
                    raise ValueError(f"Camera {cam_id} has empty img_topic in config.")
                self.image_subs.append(
                    rospy.Subscriber(
                        topic,
                        Image,
                        lambda msg, cam_id=cam_id: self.image_callback(cam_id, msg),
                        queue_size=20,
                    )
                )
        self.writer_thread.start()
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Omni-LIVO gsplat fisheye exporter started")
        rospy.loginfo("pose_topic=%s", self.pose_topic)
        rospy.loginfo("pose_frame=%s", self.pose_frame)
        rospy.loginfo("camera_ids=%s", self.camera_ids)
        if self.image_topic:
            rospy.loginfo("image_topic=%s", self.image_topic)
        rospy.loginfo("output_dir=%s", str(self.output_dir))
        rospy.loginfo("camera_id=%d", self.camera_id)
        rospy.loginfo("sync_mode=%s", self.sync_mode)
        rospy.loginfo("pose_time_offset=%.6f", self.pose_time_offset)
        rospy.loginfo("image_time_offset=%.6f", self.image_time_offset)
        rospy.loginfo("pose_stamp_source=%s", self.pose_stamp_source)
        rospy.loginfo("image_stamp_source=%s", self.image_stamp_source)
        rospy.loginfo("image_format=%s", self.image_format)
        rospy.loginfo("jpg_quality=%d", self.jpg_quality)
        rospy.loginfo("writer_queue_size=%d", self.writer_queue_size)

    def _validate_stamp_source(self, source: str, param_name: str) -> str:
        if source not in {"header", "arrival"}:
            raise ValueError(f"{param_name} must be 'header' or 'arrival', got: {source}")
        return source

    def _validate_pose_frame(self, pose_frame: str) -> str:
        if pose_frame not in {"imu", "lidar"}:
            raise ValueError(f"~pose_frame must be 'imu' or 'lidar', got: {pose_frame}")
        return pose_frame

    def _resolve_camera_ids(self) -> List[int]:
        if self.camera_ids_param:
            return [int(x.strip()) for x in self.camera_ids_param.split(",") if x.strip()]
        return [self.camera_id]

    def _select_stamp(self, header_stamp: float, arrival_stamp: float, source: str) -> float:
        if source == "arrival":
            return arrival_stamp
        return header_stamp

    def _load_yaml(self, path: Path) -> dict:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"YAML root must be a mapping: {path}")
        return data

    def _load_intrinsics(self, camera_id: int) -> Dict:
        key = f"cam_{camera_id}"
        cam = self.intrinsics_yaml.get(key)
        if cam is None:
            raise ValueError(f"Missing intrinsics entry: {key}")
        return {
            "model": str(cam["cam_model"]),
            "width": int(cam["cam_width"]),
            "height": int(cam["cam_height"]),
            "xi": float(cam["cam_xi"]),
            "fu": float(cam["cam_fu"]),
            "fv": float(cam["cam_fv"]),
            "pu": float(cam["cam_pu"]),
            "pv": float(cam["cam_pv"]),
            "distortion": [
                float(cam["cam_d0"]),
                float(cam["cam_d1"]),
                float(cam["cam_d2"]),
                float(cam["cam_d3"]),
            ],
        }

    def _load_extrinsics(self, camera_id: int) -> Dict:
        extrin = self.config["extrin_calib"]
        r_il = np.array(extrin["extrinsic_R"], dtype=np.float64).reshape(3, 3)
        t_il = np.array(extrin["extrinsic_T"], dtype=np.float64).reshape(3)
        r_li = r_il.T
        p_li = -r_il.T @ t_il

        cameras = extrin["cameras"]
        cam_node = next((cam for cam in cameras if int(cam["cam_id"]) == camera_id), None)
        if cam_node is None:
            raise ValueError(f"Camera id {camera_id} not found in extrin_calib/cameras")

        r_cl = np.array(cam_node["Rcl"], dtype=np.float64).reshape(3, 3)
        p_cl = np.array(cam_node["Pcl"], dtype=np.float64).reshape(3)
        r_ci = r_cl @ r_li
        p_ci = r_cl @ p_li + p_cl

        return {
            "camera_id": camera_id,
            "topic": str(cam_node.get("img_topic", "")),
            "r_cl": r_cl,
            "p_cl": p_cl,
            "r_ci": r_ci,
            "p_ci": p_ci,
        }

    def pose_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        header_stamp = msg.header.stamp.to_sec() + self.pose_time_offset
        arrival_stamp = rospy.get_time()
        stamp = self._select_stamp(header_stamp, arrival_stamp, self.pose_stamp_source)
        sample = PoseSample(
            stamp=stamp,
            header_stamp=header_stamp,
            arrival_stamp=arrival_stamp,
            t_wb=[pose.position.x, pose.position.y, pose.position.z],
            q_wb_xyzw=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        )
        with self.lock:
            self.pose_buffer.append(sample)

    def _warn_pose_mismatch(self, image_stamp: float, best_stamp: Optional[float], delta: Optional[float]) -> None:
        if best_stamp is None or delta is None:
            key = "empty_buffer"
            if key != self._last_sync_warning_key:
                rospy.logwarn(
                    "No pose samples received yet. image_stamp=%.6f, sync_mode=%s",
                    image_stamp,
                    self.sync_mode,
                )
                self._last_sync_warning_key = key
            return

        key = f"{self.sync_mode}:{int(delta)}"
        if key != self._last_sync_warning_key:
            rospy.logwarn(
                "No matching pose found. image_stamp=%.6f, best_pose_stamp=%.6f, delta=%.6f sec, "
                "max_pose_age=%.6f, sync_mode=%s",
                image_stamp,
                best_stamp,
                delta,
                self.max_pose_age,
                self.sync_mode,
            )
            self._last_sync_warning_key = key

    def _find_pose(self, stamp: float) -> Optional[PoseSample]:
        with self.lock:
            if not self.pose_buffer:
                self._warn_pose_mismatch(stamp, None, None)
                return None
            latest = self.pose_buffer[-1]
            best = min(self.pose_buffer, key=lambda item: abs(item.stamp - stamp))

        delta = abs(best.stamp - stamp)
        if self.sync_mode == "latest":
            return latest
        if delta > self.max_pose_age:
            self._warn_pose_mismatch(stamp, best.stamp, delta)
            return None
        return best

    def _build_camera_transforms(self, pose: PoseSample, camera_id: int) -> Tuple[np.ndarray, np.ndarray]:
        r_wb = quaternion_xyzw_to_rotation_matrix(pose.q_wb_xyzw)
        t_wb = np.array(pose.t_wb, dtype=np.float64)
        r_bw = r_wb.T

        camera_extrinsics = self.camera_extrinsics_map[camera_id]
        if self.pose_frame == "imu":
            r_cb = camera_extrinsics["r_ci"]
            p_cb = camera_extrinsics["p_ci"]
        else:
            r_cb = camera_extrinsics["r_cl"]
            p_cb = camera_extrinsics["p_cl"]

        r_cw = r_cb @ r_bw
        t_cw = -r_cb @ r_bw @ t_wb + p_cb

        r_wc = r_cw.T
        t_wc = -r_cw.T @ t_cw
        return make_transform(r_wc, t_wc), make_transform(r_cw, t_cw)

    def _writer_loop(self) -> None:
        while not self.stop_writer or not self.write_queue.empty():
            try:
                item: PendingFrame = self.write_queue.get(timeout=0.2)
            except Empty:
                continue

            try:
                save_image_with_pillow(
                    item.image, item.image_path, self.image_format, self.jpg_quality
                )
            except Exception as exc:
                rospy.logerr("Failed to write image %s: %s", str(item.image_path), str(exc))
                self.write_queue.task_done()
                continue

            with self.lock:
                self.frames.append(item.frame)
                self.saved_count += 1
                stamp = float(item.frame["timestamp"])
                if self.first_saved_stamp is None:
                    self.first_saved_stamp = stamp
                self.last_saved_stamp = stamp

            self.write_queue.task_done()

    def image_callback(self, camera_id: int, msg: Image) -> None:
        self.received_images += 1
        self.received_images_by_camera[camera_id] += 1
        if self.received_images_by_camera[camera_id] % self.save_every_n != 0:
            return

        image_header_stamp = msg.header.stamp.to_sec() + self.image_time_offset
        image_arrival_stamp = rospy.get_time()
        stamp = self._select_stamp(image_header_stamp, image_arrival_stamp, self.image_stamp_source)
        if self.first_received_stamp is None:
            self.first_received_stamp = stamp
        self.last_received_stamp = stamp
        pose = self._find_pose(stamp)
        if pose is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except CvBridgeError as exc:
            rospy.logerr("cv_bridge conversion failed: %s", str(exc))
            return

        if image is None or image.size == 0:
            rospy.logwarn("Empty image received, skipping.")
            return

        frame_stamp = stamp if self.use_header_time_for_name else rospy.Time.now().to_sec()
        frame_id = self.enqueued_images
        file_name = f"{self.image_prefix}_{frame_id:06d}_{frame_stamp:.6f}.{self.image_format}"
        image_path = self.images_dir / file_name

        c2w, w2c = self._build_camera_transforms(pose, camera_id)
        frame = {
            "frame_id": frame_id,
            "camera_id": camera_id,
            "file_path": f"images/{file_name}",
            "image_path": str(image_path.resolve()),
            "transform_matrix": c2w.tolist(),
            "w2c": w2c.tolist(),
            "timestamp": float(stamp),
            "image_header_timestamp": float(image_header_stamp),
            "image_arrival_timestamp": float(image_arrival_stamp),
            "pose_timestamp": float(pose.stamp),
            "pose_header_timestamp": float(pose.header_stamp),
            "pose_arrival_timestamp": float(pose.arrival_stamp),
            "time_diff": float(abs(stamp - pose.stamp)),
            "sync_time_source": {
                "image": self.image_stamp_source,
                "pose": self.pose_stamp_source,
            },
        }

        try:
            self.write_queue.put_nowait(PendingFrame(image=image.copy(), image_path=image_path, frame=frame))
            self.enqueued_images = frame_id + 1
        except Full:
            self.dropped_images += 1
            rospy.logwarn_throttle(
                2.0,
                "Writer queue full, dropping frame. dropped=%d queue=%d/%d",
                self.dropped_images,
                self.write_queue.qsize(),
                self.writer_queue_size,
            )
            return

        rospy.loginfo_throttle(
            1.0,
            "Received=%d Enqueued=%d Saved=%d Dropped=%d Queue=%d/%d",
            self.received_images,
            self.enqueued_images,
            self.saved_count,
            self.dropped_images,
            self.write_queue.qsize(),
            self.writer_queue_size,
        )

    def _build_transforms_payload(self) -> Dict:
        first_cam_id = self.camera_ids[0]
        intr = self.intrinsics_map[first_cam_id]
        camera_section = {
            "camera_id": first_cam_id,
            "camera_model": "OPENCV_FISHEYE",
            "width": intr["width"],
            "height": intr["height"],
            "w": intr["width"],
            "h": intr["height"],
            "fl_x": intr["fu"],
            "fl_y": intr["fv"],
            "cx": intr["pu"],
            "cy": intr["pv"],
            "k1": intr["distortion"][0],
            "k2": intr["distortion"][1],
            "k3": intr["distortion"][2],
            "k4": intr["distortion"][3],
        }

        cameras_section = {}
        for cam_id in self.camera_ids:
            intr = self.intrinsics_map[cam_id]
            extr = self.camera_extrinsics_map[cam_id]
            cameras_section[str(cam_id)] = {
                "camera_id": cam_id,
                "topic": extr["topic"],
                "camera_model": "OPENCV_FISHEYE",
                "width": intr["width"],
                "height": intr["height"],
                "w": intr["width"],
                "h": intr["height"],
                "fl_x": intr["fu"],
                "fl_y": intr["fv"],
                "cx": intr["pu"],
                "cy": intr["pv"],
                "k1": intr["distortion"][0],
                "k2": intr["distortion"][1],
                "k3": intr["distortion"][2],
                "k4": intr["distortion"][3],
                "source_intrinsics": {
                    "camera_model": intr["model"],
                    "xi": intr["xi"],
                    "fu": intr["fu"],
                    "fv": intr["fv"],
                    "pu": intr["pu"],
                    "pv": intr["pv"],
                    "distortion_d0_d1_d2_d3": intr["distortion"],
                },
                "omni_livo_extrinsics": {
                    "r_cl": extr["r_cl"].tolist(),
                    "p_cl": extr["p_cl"].tolist(),
                    "r_ci": extr["r_ci"].tolist(),
                    "p_ci": extr["p_ci"].tolist(),
                },
            }

        meta = {
            "dataset_type": "omni_livo_gsplat_export",
            "pose_topic": self.pose_topic,
            "image_topic": self.image_topic if self.image_topic else "multi-camera",
            "output_dir": str(self.output_dir.resolve()),
            "camera_id": first_cam_id,
            "camera_ids": self.camera_ids,
            "save_every_n": self.save_every_n,
            "max_pose_age": self.max_pose_age,
            "pose_frame": self.pose_frame,
            "pose_stamp_source": self.pose_stamp_source,
            "image_stamp_source": self.image_stamp_source,
            "pose_convention": f"input pose is {self.pose_frame}-in-world from ROS odometry",
            "transform_convention": {
                "transform_matrix": "camera_to_world",
                "w2c": "world_to_camera",
            },
            "source_intrinsics": cameras_section[str(first_cam_id)]["source_intrinsics"],
            "fisheye_export_assumption": (
                "Omni-LIVO d0..d3 are exported as OPENCV_FISHEYE k1..k4. "
                "The original xi parameter is preserved only in metadata."
            ),
            "omni_livo_extrinsics": cameras_section[str(first_cam_id)]["omni_livo_extrinsics"],
        }

        with self.lock:
            frames = list(self.frames)

        return {
            **camera_section,
            "cameras": cameras_section,
            "meta": meta,
            "frames": frames,
        }

    def write_transforms_json(self) -> None:
        output_path = self.output_dir / "transforms.json"
        payload = self._build_transforms_payload()
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        rospy.loginfo("Wrote transforms file: %s", str(output_path))

    def on_shutdown(self) -> None:
        try:
            self.stop_writer = True
            self.writer_thread.join(timeout=30.0)
            self.write_transforms_json()
            received_span = (
                (self.last_received_stamp - self.first_received_stamp)
                if self.first_received_stamp is not None and self.last_received_stamp is not None
                else 0.0
            )
            saved_span = (
                (self.last_saved_stamp - self.first_saved_stamp)
                if self.first_saved_stamp is not None and self.last_saved_stamp is not None
                else 0.0
            )
            recv_hz = (self.received_images / received_span) if received_span > 0 else 0.0
            save_hz = (self.saved_count / saved_span) if saved_span > 0 else 0.0
            rospy.loginfo(
                "Exporter shutdown complete. received=%d enqueued=%d saved=%d dropped=%d recv_hz=%.3f save_hz=%.3f",
                self.received_images,
                self.enqueued_images,
                self.saved_count,
                self.dropped_images,
                recv_hz,
                save_hz,
            )
        except Exception as exc:
            rospy.logerr("Failed to finalize export: %s", str(exc))


def main() -> None:
    exporter = OmniLIVOGsplatExporter()
    rospy.spin()


if __name__ == "__main__":
    main()
