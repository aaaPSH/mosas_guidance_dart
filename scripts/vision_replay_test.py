#!/usr/bin/env python3
"""vision_replay 的无 GUI 单元测试。"""

import unittest
from pathlib import Path

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

    def test_roi_is_clipped_before_processing(self):
        result = VisionReplayPipeline(
            make_params(
                initial_roi_x=-10,
                initial_roi_y=-5,
                initial_roi_width=100,
                initial_roi_height=80,
            )
        ).process(green_frame())

        self.assertEqual(result.roi, (0, 0, 64, 48))

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


if __name__ == "__main__":
    unittest.main()
