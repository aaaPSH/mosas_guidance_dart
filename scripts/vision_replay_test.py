#!/usr/bin/env python3
"""vision_replay 的无 GUI 单元测试。"""

import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import cv2
import numpy as np

import vision_replay
from vision_replay import (
    PipelineResult,
    VisionBlob,
    VisionParams,
    VisionReplayPipeline,
    load_vision_params,
)


def make_params(**overrides):
    values = {
        "initial_roi_x": 0,
        "initial_roi_y": 0,
        "initial_roi_width": 64,
        "initial_roi_height": 48,
        "h_min": 35,
        "h_max": 85,
        "s_min": 80,
        "s_max": 255,
        "v_min": 100,
        "v_max": 255,
        "found_range_x": 4,
        "found_range_y": 4,
        "min_blob_area": 8,
        "min_aspect_ratio": 0.5,
        "min_fill_ratio": 0.35,
    }
    values.update(overrides)
    return VisionParams(**values)


def green_frame():
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    cv2.circle(frame, (32, 24), 5, (0, 255, 0), -1)
    return frame


class VisionReplayPipelineTest(unittest.TestCase):
    def setUp(self):
        self.pipeline = VisionReplayPipeline(make_params())

    def test_black_frame_reports_only_target_absence(self):
        result = self.pipeline.process(np.zeros((48, 64, 3), dtype=np.uint8))

        self.assertFalse(result.found)
        self.assertEqual(result.next_roi, (0, 0, 64, 48))
        self.assertFalse(hasattr(result, "stage"))
        self.assertFalse(hasattr(result, "mask_pixel_count"))
        self.assertFalse(hasattr(result, "component_count"))
        self.assertFalse(hasattr(result, "candidate_count"))
        self.assertFalse(hasattr(result, "debug_blob"))

    def test_green_target_is_found_and_expands_roi(self):
        result = self.pipeline.process(green_frame())

        self.assertTrue(result.found)
        self.assertGreater(result.blob.area, 0)
        self.assertGreaterEqual(result.next_roi[2], result.blob.width)
        self.assertGreaterEqual(result.next_roi[3], result.blob.height)
        self.assertFalse(hasattr(result, "components"))
        self.assertFalse(hasattr(result, "candidates"))

    def test_rejected_target_is_not_exposed_as_diagnostic_blob(self):
        result = VisionReplayPipeline(
            make_params(min_blob_area=1000)
        ).process(green_frame())

        self.assertFalse(result.found)
        self.assertEqual(result.next_roi, (0, 0, 64, 48))
        self.assertFalse(hasattr(result, "debug_blob"))

    def test_inverted_hue_threshold_does_not_silently_swap_bounds(self):
        result = VisionReplayPipeline(
            make_params(h_min=85, h_max=35)
        ).process(green_frame())

        self.assertFalse(result.found)

    def test_global_search_is_not_limited_by_initial_roi(self):
        result = VisionReplayPipeline(
            make_params(
                initial_roi_x=-10,
                initial_roi_y=-5,
                initial_roi_width=100,
                initial_roi_height=80,
            )
        ).process(green_frame())

        self.assertTrue(result.found)
        self.assertEqual(result.next_roi, (23, 15, 19, 19))

    def test_result_view_keeps_only_roi_target_and_detection_status(self):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        renderer = getattr(vision_replay, "render_result_view", None)
        self.assertIsNotNone(renderer)
        status_text = getattr(vision_replay, "target_status_text", None)
        self.assertIsNotNone(status_text)
        self.assertEqual(status_text(False), "Target: NOT DETECTED")
        self.assertEqual(status_text(True), "Target: DETECTED")
        absent = renderer(frame, PipelineResult())
        detected = renderer(
            frame,
            PipelineResult(
                found=True,
                blob=VisionBlob(
                    x=20,
                    y=15,
                    width=10,
                    height=10,
                    area=80,
                    center_x=25.0,
                    center_y=20.0,
                ),
                next_roi=(16, 11, 18, 18),
            ),
        )

        self.assertEqual(absent.shape, frame.shape)
        self.assertEqual(detected.shape, frame.shape)
        self.assertFalse(np.array_equal(absent, detected))

    def test_config_loader_reads_guidance_vision_values(self):
        params = load_vision_params(
            Path(__file__).parents[1]
            / "config"
            / "mosas_guidance.conf"
        )

        self.assertEqual(params.h_max, 100)
        self.assertEqual(params.s_min, 80)
        self.assertEqual(params.v_min, 100)
        self.assertEqual(params.min_blob_area, 8)


class RecordedOverlayReplayTest(unittest.TestCase):
    def test_closed_replay_window_ends_current_video(self):
        with patch.object(
            cv2, "setTrackbarPos", side_effect=cv2.error("trackbar")
        ):
            self.assertFalse(vision_replay.sync_frame_trackbar(0))

    def test_overlay_mask_excludes_recorded_green_graphics_before_recognition(self):
        frame = np.zeros((72, 96, 3), dtype=np.uint8)
        frame[2:15, 2:17] = (0, 255, 0)
        frame[65:72, 2:40] = (0, 255, 0)
        cv2.circle(frame, (48, 36), 3, (0, 255, 0), 1)
        cv2.circle(frame, (78, 35), 5, (0, 255, 0), -1)

        mask = vision_replay.OverlayMask(20, 18, 9, 48, 36, 6)
        recognition_frame = vision_replay.mask_recorded_overlays(frame, mask)
        result = VisionReplayPipeline(
            make_params(initial_roi_width=96, initial_roi_height=72)
        ).process(recognition_frame)

        self.assertEqual(tuple(frame[5, 5]), (0, 255, 0))
        self.assertEqual(tuple(recognition_frame[5, 5]), (0, 0, 0))
        self.assertEqual(tuple(recognition_frame[68, 10]), (0, 0, 0))
        self.assertEqual(tuple(recognition_frame[36, 48]), (0, 0, 0))
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.blob.center_x, 78, delta=2)

    def test_seek_and_parameter_change_recompute_tracking_from_first_frame(self):
        with TemporaryDirectory() as directory:
            video_path = str(Path(directory) / "tracking.avi")
            writer = cv2.VideoWriter(
                video_path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (96, 72)
            )
            self.assertTrue(writer.isOpened())
            for x in (20, 65, 65):
                frame = np.zeros((72, 96, 3), dtype=np.uint8)
                cv2.circle(frame, (x, 36), 5, (0, 255, 0), -1)
                writer.write(frame)
            writer.release()

            capture = cv2.VideoCapture(video_path)
            self.assertTrue(capture.isOpened())
            try:
                params = make_params(
                    initial_roi_width=96,
                    initial_roi_height=72,
                    found_range_x=2,
                    found_range_y=2,
                )
                replay = vision_replay.ReplaySession(
                    capture, params, vision_replay.OverlayMask(0, 0, 0, 0, 0, 0)
                )
                self.assertTrue(replay.seek(1))
                self.assertFalse(replay.current_result.found)
                self.assertTrue(replay.seek(2))
                self.assertTrue(replay.current_result.found)
                self.assertEqual(replay.history_found, [True, False, True])
                self.assertTrue(replay.seek(1))
                self.assertFalse(replay.current_result.found)
                self.assertEqual(replay.history_found, [True, False])

                replay.update_params(replace(params, found_range_x=50))
                self.assertTrue(replay.seek(1))
                self.assertTrue(replay.current_result.found)
                self.assertEqual(replay.frame_index, 1)
            finally:
                capture.release()

    def test_default_mask_uses_recording_reference_point(self):
        mask = vision_replay.load_overlay_mask(
            Path(__file__).parents[1] / "config" / "mosas_guidance.conf",
            640,
            480,
            8,
        )
        self.assertEqual((mask.reference_x, mask.reference_y), (320, 240))
        self.assertGreaterEqual(mask.top_left_width, 330)
        self.assertGreaterEqual(mask.top_left_height, 95)
        self.assertGreaterEqual(mask.bottom_height, 20)

    def test_status_marker_overrides_occluded_new_detection(self):
        detected = np.zeros((48, 160, 3), dtype=np.uint8)
        cv2.putText(
            detected, "Target: DETECTED", (6, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
        )
        missed = np.zeros((48, 160, 3), dtype=np.uint8)
        cv2.putText(
            missed, "Target: NOT DETECTED", (6, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA,
        )
        blank = np.zeros((48, 160, 3), dtype=np.uint8)

        self.assertIs(vision_replay.detect_recorded_status_marker(detected), True)
        self.assertIs(vision_replay.detect_recorded_status_marker(missed), False)
        self.assertIsNone(vision_replay.detect_recorded_status_marker(blank))

    def test_mask_reference_point_reads_camera_intrinsics(self):
        with TemporaryDirectory() as directory:
            config_path = Path(directory) / "camera.conf"
            config_path.write_text(
                "[camera]\nmatrix_m02=22\nmatrix_m12=27\n[guidance]\n",
                encoding="utf-8",
            )
            mask = vision_replay.load_overlay_mask(config_path, 96, 72, 8)
            self.assertEqual((mask.reference_x, mask.reference_y), (22, 27))

    def test_session_starts_at_first_frame_after_preview_read(self):
        with TemporaryDirectory() as directory:
            video_path = str(Path(directory) / "preview.avi")
            writer = cv2.VideoWriter(
                video_path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (96, 72)
            )
            self.assertTrue(writer.isOpened())
            for x in (20, 65):
                frame = np.zeros((72, 96, 3), dtype=np.uint8)
                cv2.circle(frame, (x, 36), 5, (0, 255, 0), -1)
                writer.write(frame)
            writer.release()
            capture = cv2.VideoCapture(video_path)
            try:
                self.assertTrue(capture.read()[0])
                replay = vision_replay.ReplaySession(
                    capture,
                    make_params(initial_roi_width=96, initial_roi_height=72),
                    vision_replay.OverlayMask(0, 0, 0, 0, 0, 0),
                )
                self.assertTrue(replay.seek(0))
                self.assertAlmostEqual(replay.current_result.blob.center_x, 20, delta=2)
            finally:
                capture.release()

    def test_changing_mask_reprocesses_current_frame(self):
        with TemporaryDirectory() as directory:
            video_path = str(Path(directory) / "overlay.avi")
            writer = cv2.VideoWriter(
                video_path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (96, 72)
            )
            self.assertTrue(writer.isOpened())
            frame = np.zeros((72, 96, 3), dtype=np.uint8)
            cv2.rectangle(frame, (2, 2), (16, 16), (0, 255, 0), -1)
            writer.write(frame)
            writer.write(frame)
            writer.release()
            capture = cv2.VideoCapture(video_path)
            try:
                replay = vision_replay.ReplaySession(
                    capture,
                    make_params(initial_roi_width=96, initial_roi_height=72),
                    vision_replay.OverlayMask(0, 0, 0, 0, 0, 0),
                )
                self.assertTrue(replay.seek(0))
                self.assertTrue(replay.current_result.found)
                replay.update_mask(vision_replay.OverlayMask(20, 20, 0, 0, 0, 0))
                self.assertTrue(replay.seek(0))
                self.assertFalse(replay.current_result.found)
            finally:
                capture.release()

    def test_recorded_target_overlap_marks_masked_frame(self):
        with TemporaryDirectory() as directory:
            video_path = Path(directory) / "flight.avi"
            video_path.with_suffix(".csv").write_text(
                "frame_index,vision_found,target_x,target_y,target_width,target_height\n"
                "0,1,10,12,7,7\n1,1,70,30,7,7\n",
                encoding="utf-8",
            )
            recorded = vision_replay.load_recorded_detections(video_path)
            mask = vision_replay.OverlayMask(30, 25, 8, 48, 36, 5)
            self.assertTrue(
                vision_replay.recorded_target_is_masked(recorded[0], mask, 96, 72)
            )
            self.assertFalse(
                vision_replay.recorded_target_is_masked(recorded[1], mask, 96, 72)
            )


if __name__ == "__main__":
    unittest.main()
