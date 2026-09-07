#!/usr/bin/env python3
"""vision_replay 的无 GUI 单元测试。"""

import unittest
from pathlib import Path

import cv2
import numpy as np

from vision_replay import (
    VisionParams,
    VisionReplayPipeline,
    VisionStage,
    create_dashboard,
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

    def test_black_frame_reports_mask_empty(self):
        result = self.pipeline.process(np.zeros((48, 64, 3), dtype=np.uint8))

        self.assertEqual(result.stage, VisionStage.MASK_EMPTY)
        self.assertEqual(result.mask_pixel_count, 0)
        self.assertFalse(result.found)
        self.assertEqual(result.component_count, 0)

    def test_green_blob_is_found_and_expands_roi(self):
        result = self.pipeline.process(green_frame())

        self.assertEqual(result.stage, VisionStage.FOUND)
        self.assertTrue(result.found)
        self.assertGreater(result.mask_pixel_count, 0)
        self.assertEqual(result.component_count, 1)
        self.assertGreaterEqual(result.candidate_count, 1)
        self.assertGreater(result.next_roi[2], result.blob.width)
        self.assertTrue(result.mask_valid)

    def test_large_min_area_reports_rejected_candidate(self):
        result = VisionReplayPipeline(
            make_params(min_blob_area=1000)
        ).process(green_frame())

        self.assertEqual(result.stage, VisionStage.CANDIDATE_REJECTED)
        self.assertTrue(result.debug_blob.valid)
        self.assertFalse(result.found)

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
        self.assertEqual(result.mask.shape, (48, 64))

    def test_threshold_change_changes_mask_pixel_count(self):
        params = make_params(h_min=0, h_max=10)
        result = VisionReplayPipeline(params).process(green_frame())

        self.assertEqual(result.stage, VisionStage.MASK_EMPTY)
        self.assertEqual(result.mask_pixel_count, 0)

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

    def test_dashboard_has_three_by_three_panels(self):
        frame = green_frame()
        result = self.pipeline.process(frame)

        dashboard = create_dashboard(frame, result, cell_size=(160, 120))

        self.assertEqual(dashboard.shape, (360, 480, 3))
        self.assertEqual(dashboard.dtype, np.uint8)


if __name__ == "__main__":
    unittest.main()
