#!/usr/bin/env python3
import math
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from omni_stitch_capture.msg import StitchedCapture


DEFAULT_OMNI_DIR = "/home/pray/omni_ws/src/Omni-LIVO/Omni-LIVO"


@dataclass
class PoseSample:
    stamp: float
    msg: Odometry


@dataclass
class CapturePoint:
    x: float
    y: float
    yaw: float
    stamp: float


@dataclass
class CandidateFrame:
    stitched: np.ndarray
    pose_msg: Odometry
    pair_stamp_sec: float
    pair_diff_ms: float
    pose_source: str
    pose_diff_sec: float
    blur_score: float


@dataclass
class PendingCapture:
    anchor_pose_msg: Odometry
    anchor_stamp_sec: float
    reason: str
    candidates: List[CandidateFrame]


class OmniStitchCaptureNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.left_topic = rospy.get_param("~left_topic", "/fisheye/left/image_raw")
        self.right_topic = rospy.get_param("~right_topic", "/fisheye/right/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/aft_mapped_to_init")
        self.output_image_topic = rospy.get_param("~output_image_topic", "/omni_stitch_capture/image_raw")
        self.output_pose_topic = rospy.get_param("~output_pose_topic", "/omni_stitch_capture/pose")
        self.output_capture_topic = rospy.get_param("~output_capture_topic", "/omni_stitch_capture/capture")
        self.publish_legacy_topics = bool(rospy.get_param("~publish_legacy_topics", False))
        self.frame_id = rospy.get_param("~frame_id", "stitched_panorama")

        self.intrinsics_path = rospy.get_param(
            "~intrinsics_path", f"{DEFAULT_OMNI_DIR}/config/camera_intrinsics.yaml"
        )
        self.config_path = rospy.get_param(
            "~config_path", f"{DEFAULT_OMNI_DIR}/config/mid360_multi_cam_right_delay.yaml"
        )

        self.left_camera_id = int(rospy.get_param("~left_camera_id", 0))
        self.right_camera_id = int(rospy.get_param("~right_camera_id", 1))
        self.max_pair_diff_sec = float(rospy.get_param("~max_pair_diff_sec", 0.01))
        self.pose_match_max_diff_sec = float(rospy.get_param("~pose_match_max_diff_sec", 0.05))
        self.pose_fallback_max_diff_sec = float(rospy.get_param("~pose_fallback_max_diff_sec", 0.50))
        self.pose_time_offset_sec = float(rospy.get_param("~pose_time_offset_sec", 0.0))
        self.auto_pose_time_offset = bool(rospy.get_param("~auto_pose_time_offset", True))
        self.auto_pose_time_offset_trigger_sec = float(
            rospy.get_param("~auto_pose_time_offset_trigger_sec", 10.0)
        )
        self.pose_buffer_size = int(rospy.get_param("~pose_buffer_size", 400))

        self.capture_spacing_m = float(rospy.get_param("~capture_spacing_m", 2.0))
        self.corner_min_separation_m = float(rospy.get_param("~corner_min_separation_m", 1.0))
        self.corner_min_travel_m = float(rospy.get_param("~corner_min_travel_m", 0.5))
        self.corner_yaw_threshold_deg = float(rospy.get_param("~corner_yaw_threshold_deg", 35.0))
        self.candidate_frame_count = int(rospy.get_param("~candidate_frame_count", 5))
        self.blur_min_laplacian_var = float(rospy.get_param("~blur_min_laplacian_var", 80.0))

        self.band_width = int(rospy.get_param("~band_width", 1920))
        self.band_height = int(rospy.get_param("~band_height", 460))
        self.canvas_height = int(rospy.get_param("~canvas_height", 960))
        self.canvas_anchor = str(rospy.get_param("~canvas_anchor", "bottom")).lower()
        self.yaw_min_deg = float(rospy.get_param("~yaw_min_deg", -110.0))
        self.yaw_max_deg = float(rospy.get_param("~yaw_max_deg", 110.0))
        self.pitch_min_deg = float(rospy.get_param("~pitch_min_deg", -35.0))
        self.pitch_max_deg = float(rospy.get_param("~pitch_max_deg", 20.0))
        self.max_incidence_deg = float(rospy.get_param("~max_incidence_deg", 105.0))
        self.edge_blend_px = float(rospy.get_param("~edge_blend_px", 40.0))

        self.pose_buffer: Deque[PoseSample] = deque(maxlen=self.pose_buffer_size)
        self.capture_points: List[CapturePoint] = []
        self.last_pose_msg: Optional[Odometry] = None
        self.pending_capture: Optional[PendingCapture] = None

        self.left_cfg = self._load_camera_config(self.left_camera_id)
        self.right_cfg = self._load_camera_config(self.right_camera_id)
        self.left_map_x, self.left_map_y, self.left_weight = self._build_camera_projection(self.left_cfg)
        self.right_map_x, self.right_map_y, self.right_weight = self._build_camera_projection(self.right_cfg)

        self.capture_pub = rospy.Publisher(self.output_capture_topic, StitchedCapture, queue_size=3)
        self.image_pub = None
        self.pose_pub = None
        if self.publish_legacy_topics:
            self.image_pub = rospy.Publisher(self.output_image_topic, Image, queue_size=3)
            self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=3)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=200)

        left_sub = message_filters.Subscriber(self.left_topic, Image)
        right_sub = message_filters.Subscriber(self.right_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=20, slop=self.max_pair_diff_sec
        )
        self.sync.registerCallback(self._image_pair_callback)

        rospy.loginfo(
            "omni_stitch_capture started: left=%s right=%s odom=%s spacing=%.2fm corner_yaw=%.1fdeg yaw=[%.1f, %.1f] pitch=[%.1f, %.1f] anchor=%s",
            self.left_topic,
            self.right_topic,
            self.odom_topic,
            self.capture_spacing_m,
            self.corner_yaw_threshold_deg,
            self.yaw_min_deg,
            self.yaw_max_deg,
            self.pitch_min_deg,
            self.pitch_max_deg,
            self.canvas_anchor,
        )

    def _load_yaml(self, path: str):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _load_camera_config(self, camera_id: int) -> dict:
        intr_root = self._load_yaml(self.intrinsics_path)
        cfg_root = self._load_yaml(self.config_path)

        intr = intr_root[f"cam_{camera_id}"]
        cam_cfg = None
        for item in cfg_root["extrin_calib"]["cameras"]:
            if int(item["cam_id"]) == camera_id:
                cam_cfg = item
                break
        if cam_cfg is None:
            raise RuntimeError(f"Camera config for cam_id={camera_id} not found")

        return {
            "width": int(intr["cam_width"]),
            "height": int(intr["cam_height"]),
            "xi": float(intr["cam_xi"]),
            "fu": float(intr["cam_fu"]),
            "fv": float(intr["cam_fv"]),
            "pu": float(intr["cam_pu"]),
            "pv": float(intr["cam_pv"]),
            "distortion": np.array(
                [intr["cam_d0"], intr["cam_d1"], intr["cam_d2"], intr["cam_d3"]],
                dtype=np.float64,
            ),
            "rotation_cb": np.array(cam_cfg["Rcl"], dtype=np.float64).reshape(3, 3),
        }

    def _build_projection_dirs(self) -> np.ndarray:
        yaw = np.deg2rad(np.linspace(self.yaw_max_deg, self.yaw_min_deg, self.band_width, dtype=np.float32))
        pitch = np.deg2rad(np.linspace(self.pitch_max_deg, self.pitch_min_deg, self.band_height, dtype=np.float32))
        yaw_grid, pitch_grid = np.meshgrid(yaw, pitch, indexing="xy")
        cos_pitch = np.cos(pitch_grid)
        dirs = np.stack(
            [cos_pitch * np.cos(yaw_grid), cos_pitch * np.sin(yaw_grid), np.sin(pitch_grid)],
            axis=-1,
        )
        return dirs.reshape(-1, 3).astype(np.float64)

    def _omni_project(self, points_cam: np.ndarray, cfg: dict) -> Tuple[np.ndarray, np.ndarray]:
        x = points_cam[:, 0]
        y = points_cam[:, 1]
        z = points_cam[:, 2]
        d = np.linalg.norm(points_cam, axis=1)
        denom = z + cfg["xi"] * d
        valid = denom > 1e-8

        xn = np.zeros_like(x)
        yn = np.zeros_like(y)
        xn[valid] = x[valid] / denom[valid]
        yn[valid] = y[valid] / denom[valid]

        r2 = xn * xn + yn * yn
        k1, k2, p1, p2 = cfg["distortion"]
        radial = 1.0 + k1 * r2 + k2 * r2 * r2
        x_tan = 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
        y_tan = p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn

        xd = xn * radial + x_tan
        yd = yn * radial + y_tan
        u = cfg["fu"] * xd + cfg["pu"]
        v = cfg["fv"] * yd + cfg["pv"]
        return np.stack([u, v], axis=1).astype(np.float32), valid

    def _build_camera_projection(self, cfg: dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        dirs_body = self._build_projection_dirs()
        dirs_cam = dirs_body @ cfg["rotation_cb"].T
        uv, valid = self._omni_project(dirs_cam, cfg)

        u = uv[:, 0]
        v = uv[:, 1]
        inside = valid & (u >= 0.0) & (u <= cfg["width"] - 1.0) & (v >= 0.0) & (v <= cfg["height"] - 1.0)

        z_min = math.cos(math.radians(self.max_incidence_deg))
        center_weight = np.clip((dirs_cam[:, 2] - z_min) / max(1e-6, 1.0 - z_min), 0.0, 1.0)
        edge_margin = np.minimum.reduce([u, (cfg["width"] - 1.0) - u, v, (cfg["height"] - 1.0) - v])
        edge_weight = np.clip(edge_margin / max(1.0, self.edge_blend_px), 0.0, 1.0)
        weight = np.where(inside, center_weight * edge_weight, 0.0).astype(np.float32)

        map_x = u.reshape(self.band_height, self.band_width).astype(np.float32)
        map_y = v.reshape(self.band_height, self.band_width).astype(np.float32)
        weight = weight.reshape(self.band_height, self.band_width)
        map_x[weight <= 0.0] = -1.0
        map_y[weight <= 0.0] = -1.0
        return map_x, map_y, weight

    def _odom_callback(self, msg: Odometry) -> None:
        self.pose_buffer.append(PoseSample(stamp=msg.header.stamp.to_sec(), msg=msg))
        self.last_pose_msg = msg

    def _adjust_pose_stamp(self, raw_stamp_sec: float) -> float:
        return raw_stamp_sec - self.pose_time_offset_sec

    def _maybe_auto_calibrate_pose_offset(self, pair_stamp_sec: float) -> bool:
        if not self.auto_pose_time_offset or self.last_pose_msg is None:
            return False

        raw_pose_stamp = self.last_pose_msg.header.stamp.to_sec()
        raw_diff = abs(raw_pose_stamp - pair_stamp_sec)
        if raw_diff < self.auto_pose_time_offset_trigger_sec:
            return False

        new_offset = raw_pose_stamp - pair_stamp_sec
        if abs(new_offset - self.pose_time_offset_sec) < 1e-6:
            return False

        self.pose_time_offset_sec = new_offset
        rospy.logwarn(
            "Auto calibrated pose_time_offset_sec to %.3f s using raw_pose_stamp=%.3f pair_stamp=%.3f",
            self.pose_time_offset_sec,
            raw_pose_stamp,
            pair_stamp_sec,
        )
        return True

    def _find_closest_pose(self, stamp_sec: float) -> Tuple[Optional[Odometry], Optional[float], str]:
        if not self.pose_buffer:
            return None, None, "no_pose_received"
        stamps = [self._adjust_pose_stamp(sample.stamp) for sample in self.pose_buffer]
        idx = bisect_left(stamps, stamp_sec)
        candidates = []
        if idx < len(self.pose_buffer):
            candidates.append(self.pose_buffer[idx])
        if idx > 0:
            candidates.append(self.pose_buffer[idx - 1])
        if not candidates:
            return None, None, "no_pose_candidate"
        best = min(candidates, key=lambda sample: abs(self._adjust_pose_stamp(sample.stamp) - stamp_sec))
        best_diff = abs(self._adjust_pose_stamp(best.stamp) - stamp_sec)
        if best_diff <= self.pose_match_max_diff_sec:
            return best.msg, best_diff, "nearest_pose"

        if self.last_pose_msg is not None:
            fallback_diff = abs(self._adjust_pose_stamp(self.last_pose_msg.header.stamp.to_sec()) - stamp_sec)
            if fallback_diff <= self.pose_fallback_max_diff_sec:
                return self.last_pose_msg, fallback_diff, "latest_pose_fallback"

        return None, best_diff, "pose_too_far"

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    @staticmethod
    def _dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _pose_xy(odom: Odometry) -> Tuple[float, float]:
        return float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)

    def _should_capture(self, odom: Odometry) -> Tuple[bool, str]:
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)
        current_xy = (x, y)

        if not self.capture_points:
            return True, "first_sample"

        nearest = min(self.capture_points, key=lambda item: self._dist_xy(current_xy, (item.x, item.y)))
        min_dist = self._dist_xy(current_xy, (nearest.x, nearest.y))
        if min_dist >= self.capture_spacing_m:
            return True, f"distance={min_dist:.2f}m"

        last = self.capture_points[-1]
        dist_last = self._dist_xy(current_xy, (last.x, last.y))
        yaw_diff_nearest = abs(self._angle_diff(yaw, nearest.yaw))
        yaw_diff_last = abs(self._angle_diff(yaw, last.yaw))
        yaw_diff_deg = math.degrees(max(yaw_diff_nearest, yaw_diff_last))

        if (
            min_dist >= self.corner_min_separation_m
            and dist_last >= self.corner_min_travel_m
            and yaw_diff_deg >= self.corner_yaw_threshold_deg
        ):
            return True, f"corner min_dist={min_dist:.2f}m yaw_diff={yaw_diff_deg:.1f}deg"

        return False, f"skip min_dist={min_dist:.2f}m yaw_diff={yaw_diff_deg:.1f}deg"

    def _remember_capture(self, odom: Odometry, stamp_sec: float) -> None:
        self.capture_points.append(
            CapturePoint(
                x=float(odom.pose.pose.position.x),
                y=float(odom.pose.pose.position.y),
                yaw=self._yaw_from_quaternion(odom.pose.pose.orientation),
                stamp=stamp_sec,
            )
        )

    def _stitch(self, left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
        warped_left = cv2.remap(
            left_bgr,
            self.left_map_x,
            self.left_map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        warped_right = cv2.remap(
            right_bgr,
            self.right_map_x,
            self.right_map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        use_left = self.left_weight >= self.right_weight
        valid = (self.left_weight > 0.0) | (self.right_weight > 0.0)

        band = np.zeros_like(warped_left)
        band[use_left] = warped_left[use_left]
        band[~use_left] = warped_right[~use_left]
        band[~valid] = 0

        canvas = np.zeros((self.canvas_height, self.band_width, 3), dtype=np.uint8)
        if self.canvas_anchor == "top":
            offset_y = 0
        elif self.canvas_anchor == "center":
            offset_y = max(0, (self.canvas_height - self.band_height) // 2)
        else:
            offset_y = self.canvas_height - self.band_height
        canvas[offset_y : offset_y + self.band_height, :, :] = band
        return canvas

    @staticmethod
    def _compute_blur_score(image_bgr: np.ndarray) -> float:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _publish_capture(self, candidate: CandidateFrame, reason: str) -> None:
        header_stamp = rospy.Time.from_sec(candidate.pair_stamp_sec)
        sample_index = len(self.capture_points) + 1

        out_img = self.bridge.cv2_to_imgmsg(candidate.stitched, encoding="bgr8")
        out_img.header.stamp = header_stamp
        out_img.header.frame_id = self.frame_id

        capture_msg = StitchedCapture()
        capture_msg.header.stamp = header_stamp
        capture_msg.header.frame_id = candidate.pose_msg.header.frame_id or "map"
        capture_msg.image = out_img
        capture_msg.pose = candidate.pose_msg.pose.pose
        capture_msg.sample_index = sample_index
        capture_msg.pair_diff_ms = candidate.pair_diff_ms
        capture_msg.reason = reason
        self.capture_pub.publish(capture_msg)

        if self.publish_legacy_topics and self.image_pub is not None and self.pose_pub is not None:
            self.image_pub.publish(out_img)
            pose_out = PoseStamped()
            pose_out.header.stamp = header_stamp
            pose_out.header.frame_id = capture_msg.header.frame_id
            pose_out.pose = capture_msg.pose
            self.pose_pub.publish(pose_out)

        self._remember_capture(candidate.pose_msg, candidate.pair_stamp_sec)
        rospy.loginfo(
            "Published stitched sample #%d reason=%s pair_diff=%.2fms pose_source=%s pose_diff=%.3fs blur=%.2f pose=(%.3f, %.3f, %.3f)",
            sample_index,
            reason,
            candidate.pair_diff_ms,
            candidate.pose_source,
            candidate.pose_diff_sec,
            candidate.blur_score,
            capture_msg.pose.position.x,
            capture_msg.pose.position.y,
            capture_msg.pose.position.z,
        )

    def _finalize_pending_capture(self) -> None:
        if self.pending_capture is None or not self.pending_capture.candidates:
            self.pending_capture = None
            return

        anchor_xy = self._pose_xy(self.pending_capture.anchor_pose_msg)
        sharp_candidates = [
            candidate
            for candidate in self.pending_capture.candidates
            if candidate.blur_score >= self.blur_min_laplacian_var
        ]
        selection_pool = sharp_candidates if sharp_candidates else self.pending_capture.candidates

        best_candidate = min(
            selection_pool,
            key=lambda candidate: (
                self._dist_xy(self._pose_xy(candidate.pose_msg), anchor_xy),
                -candidate.blur_score,
            ),
        )

        blur_rejected = len(self.pending_capture.candidates) - len(sharp_candidates)
        publish_reason = (
            f"{self.pending_capture.reason}; window={len(self.pending_capture.candidates)};"
            f" blur_rejected={blur_rejected}"
        )
        if not sharp_candidates:
            publish_reason += "; fallback=sharpest_available"

        self._publish_capture(best_candidate, publish_reason)
        self.pending_capture = None

    def _image_pair_callback(self, left_msg: Image, right_msg: Image) -> None:
        pair_stamp_sec = 0.5 * (left_msg.header.stamp.to_sec() + right_msg.header.stamp.to_sec())
        pose_msg, pose_diff_sec, pose_source = self._find_closest_pose(pair_stamp_sec)
        if pose_msg is None and pose_source == "pose_too_far" and self._maybe_auto_calibrate_pose_offset(pair_stamp_sec):
            pose_msg, pose_diff_sec, pose_source = self._find_closest_pose(pair_stamp_sec)
        if pose_msg is None:
            if pose_diff_sec is None:
                rospy.logwarn_throttle(
                    5.0,
                    "No matching pose found for stitched image pair: source=%s pair_stamp=%.3f pose_offset=%.3f",
                    pose_source,
                    pair_stamp_sec,
                    self.pose_time_offset_sec,
                )
            else:
                rospy.logwarn_throttle(
                    5.0,
                    "No matching pose found for stitched image pair: source=%s pair_stamp=%.3f nearest_diff=%.3f s pose_offset=%.3f",
                    pose_source,
                    pair_stamp_sec,
                    pose_diff_sec,
                    self.pose_time_offset_sec,
                )
            return

        should_capture, reason = self._should_capture(pose_msg)
        if self.pending_capture is None and not should_capture:
            return

        try:
            left_bgr = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_bgr = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr("cv_bridge conversion failed: %s", exc)
            return

        stitched = self._stitch(left_bgr, right_bgr)
        pair_diff_ms = abs(left_msg.header.stamp.to_sec() - right_msg.header.stamp.to_sec()) * 1000.0
        blur_score = self._compute_blur_score(stitched)

        if self.pending_capture is None:
            self.pending_capture = PendingCapture(
                anchor_pose_msg=pose_msg,
                anchor_stamp_sec=pair_stamp_sec,
                reason=reason,
                candidates=[],
            )
            rospy.loginfo(
                "Started candidate window reason=%s target_pose=(%.3f, %.3f, %.3f) need=%d",
                reason,
                pose_msg.pose.pose.position.x,
                pose_msg.pose.pose.position.y,
                pose_msg.pose.pose.position.z,
                self.candidate_frame_count,
            )

        self.pending_capture.candidates.append(
            CandidateFrame(
                stitched=stitched,
                pose_msg=pose_msg,
                pair_stamp_sec=pair_stamp_sec,
                pair_diff_ms=pair_diff_ms,
                pose_source=pose_source,
                pose_diff_sec=0.0 if pose_diff_sec is None else pose_diff_sec,
                blur_score=blur_score,
            )
        )

        rospy.loginfo(
            "Collected candidate %d/%d blur=%.2f pair_diff=%.2fms pose=(%.3f, %.3f)",
            len(self.pending_capture.candidates),
            self.candidate_frame_count,
            blur_score,
            pair_diff_ms,
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
        )

        if len(self.pending_capture.candidates) >= self.candidate_frame_count:
            self._finalize_pending_capture()


def main() -> None:
    rospy.init_node("omni_stitch_capture_node")
    OmniStitchCaptureNode()
    rospy.spin()


if __name__ == "__main__":
    main()
