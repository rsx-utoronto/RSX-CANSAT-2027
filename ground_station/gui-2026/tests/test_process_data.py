import shutil
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from data.process import DataProcessor  # noqa: E402
from gui.gps_map import GPSMapWidget  # noqa: E402
from gui.payload_visualization import PayloadVisualizationCanvas  # noqa: E402
from serial.payload_sim import PayloadSim  # noqa: E402


class AttitudeIndicatorStub:
    def __init__(self):
        self.roll_updates = []
        self.pitch_updates = []
        self.yaw_updates = []

    def set_roll(self, value):
        self.roll_updates.append(value)

    def set_pitch(self, value):
        self.pitch_updates.append(value)

    def set_yaw(self, value):
        self.yaw_updates.append(value)


class GraphStub:
    def __init__(self):
        self.attitude_indicator = AttitudeIndicatorStub()
        self.mode_updates = []
        self.state_updates = []
        self.flight_ctrl_updates = []
        self.attitude_updates = []
        self.voltage_updates = []
        self.current_updates = []
        self.accel_graph_updates = []
        self.accel_xyz_graph_updates = []
        self.packet_count = 0
        self.altitude_updates = []
        self.packet_label_updates = []
        self.temp_updates = []
        self.pressure_updates = []
        self.gyro_updates = []
        self.gps_map_updates = []
        self.gps_alt_updates = []
        self.mission_time_updates = []
        self.gps_time_updates = []
        self.sat_updates = []
        self.cmd_echo_updates = []
        self.camera1_updates = []
        self.camera2_updates = []

    def update_packet_count(self):
        self.packet_count += 1

    def update_mode(self, value):
        self.mode_updates.append(value)

    def update_alt_graph(self, value):
        self.altitude_updates.append(value)

    def update_state(self, value):
        self.state_updates.append(value)

    def update_flight_ctrl(self, value):
        self.flight_ctrl_updates.append(value)

    def update_attitude(self, roll, pitch, yaw):
        self.attitude_updates.append((roll, pitch, yaw))

    def update_volt_graph(self, value):
        self.voltage_updates.append(value)

    def update_current_graph(self, value):
        self.current_updates.append(value)

    def update_accel_graph(self, value):
        self.accel_graph_updates.append(value)

    def update_accel_xyz_graph(self, value):
        self.accel_xyz_graph_updates.append(value)

    def update_packet_label(self):
        self.packet_label_updates.append((self.packet_count, len(self.packet_label_updates)))

    def update_temp(self, value):
        self.temp_updates.append(value)

    def update_pressure(self, value):
        self.pressure_updates.append(value)

    def update_gyro_graph(self, value):
        self.gyro_updates.append(value)

    def update_gps_map(self, lat, lon):
        self.gps_map_updates.append((lat, lon))

    def update_gps_alt(self, value):
        self.gps_alt_updates.append(value)

    def update_mission_time(self, value):
        self.mission_time_updates.append(value)

    def update_packets_sent(self, value):
        return None

    def update_gps_time(self, value):
        self.gps_time_updates.append(value)

    def update_sats(self, value):
        self.sat_updates.append(value)

    def update_cmd_echo(self, value):
        self.cmd_echo_updates.append(value)

    def update_camera1_status(self, value):
        self.camera1_updates.append(value)

    def update_camera2_status(self, value):
        self.camera2_updates.append(value)

    def get_packet_count(self):
        return self.packet_count


class SimpStub:
    def simp_enable(self):
        return None


class DataProcessorResponseTests(unittest.TestCase):
    def make_processor(self):
        processor = DataProcessor(GraphStub(), SimpStub())
        self.addCleanup(processor.close_csv)
        self.addCleanup(processor.close_logfile)
        return processor

    def test_extract_data_str_parses_new_tail_fields(self):
        processor = self.make_processor()
        msg = (
            "1011,12:34:56,42,F,ASCENT,123.4,22.5,101.3,11.9,350.0,"
            "1.1,2.2,3.3,4.4,5.5,6.6,12:34:57,456.7,45.501,-73.567,8,"
            "CMD_OK,3,AUTONOMOUS,0.1,0.2,0.3,0.4,7.1,8.2,9.3,10.4,11.5,12.6"
        )

        data = processor.extract_data_str(msg)

        self.assertEqual(data.FLIGHT_CTRL, "AUTONOMOUS")
        self.assertEqual(data.QUATERNION_W, 0.1)
        self.assertEqual(data.QUATERNION_X, 0.2)
        self.assertEqual(data.QUATERNION_Y, 0.3)
        self.assertEqual(data.QUATERNION_Z, 0.4)
        self.assertEqual(data.VELOCITY_X, 7.1)
        self.assertEqual(data.VELOCITY_Y, 8.2)
        self.assertEqual(data.VELOCITY_Z, 9.3)
        self.assertEqual(data.ACCEL_X, 10.4)
        self.assertEqual(data.ACCEL_Y, 11.5)
        self.assertEqual(data.ACCEL_Z, 12.6)

    def test_online_message_accepts_two_field_mission_info(self):
        processor = self.make_processor()

        processor.process_data("$ MSG:CANSAT IS ONLINE.{F|IDLE}")

        self.assertEqual(processor._graph_ui.mode_updates, ["F"])
        self.assertEqual(processor._graph_ui.state_updates, ["IDLE"])
        self.assertEqual(processor._graph_ui.flight_ctrl_updates, [])

    def test_response_still_accepts_three_field_mission_info(self):
        processor = self.make_processor()

        processor.process_data("$ MSG:STATUS UPDATED.{S|ASCENT|MANUAL}")

        self.assertEqual(processor._graph_ui.mode_updates, ["S"])
        self.assertEqual(processor._graph_ui.state_updates, ["ASCENT"])
        self.assertEqual(processor._graph_ui.flight_ctrl_updates, ["MANUAL"])

    def test_parse_telemetry_routes_xyz_acceleration_to_new_graph(self):
        processor = self.make_processor()
        msg = (
            "1011,12:34:56,42,F,ASCENT,123.4,22.5,101.3,11.9,350.0,"
            "1.1,2.2,3.3,4.4,5.5,6.6,12:34:57,456.7,45.501,-73.567,8,"
            "CMD_OK,3,AUTONOMOUS,0.1,0.2,0.3,0.4,7.1,8.2,9.3,10.4,11.5,12.6"
        )

        processor.parse_telemetry_string(msg)

        self.assertEqual(processor._graph_ui.voltage_updates, [11.9])
        self.assertEqual(processor._graph_ui.current_updates, [350.0])
        self.assertEqual(processor._graph_ui.accel_graph_updates, [[4.4, 5.5, 6.6]])
        self.assertEqual(processor._graph_ui.accel_xyz_graph_updates, [[10.4, 11.5, 12.6]])

    def test_parse_telemetry_routes_quaternion_to_attitude_indicator(self):
        processor = self.make_processor()
        msg = (
            "1011,12:34:56,42,F,ASCENT,123.4,22.5,101.3,11.9,350.0,"
            "1.1,2.2,3.3,4.4,5.5,6.6,12:34:57,456.7,45.501,-73.567,8,"
            "CMD_OK,3,AUTONOMOUS,0.9515,0.0381,0.1893,0.2393,7.1,8.2,9.3,10.4,11.5,12.6"
        )

        processor.parse_telemetry_string(msg)

        self.assertEqual(len(processor._graph_ui.attitude_updates), 1)
        roll, pitch, yaw = processor._graph_ui.attitude_updates[0]
        self.assertAlmostEqual(roll, 10.0, delta=0.05)
        self.assertAlmostEqual(pitch, 20.0, delta=0.05)
        self.assertAlmostEqual(yaw, 30.0, delta=0.05)
        self.assertEqual(processor._graph_ui.attitude_indicator.roll_updates, [])
        self.assertEqual(processor._graph_ui.attitude_indicator.pitch_updates, [])
        self.assertEqual(processor._graph_ui.attitude_indicator.yaw_updates, [])


class PayloadSimTests(unittest.TestCase):
    def test_generated_telemetry_includes_uav_tail_fields(self):
        sim = PayloadSim()
        processor = DataProcessor(GraphStub(), SimpStub())
        self.addCleanup(processor.close_csv)
        self.addCleanup(processor.close_logfile)

        sim.transmitting = True
        sim.calibrated = True
        sim._generate_telemetry()

        packet = sim._buffer[-1]
        fields = packet.split(",")
        data = processor.extract_data_str(packet)

        self.assertEqual(len(fields), 34)
        self.assertEqual(data.FLIGHT_CTRL, "AUTONOMOUS")
        self.assertIsNotNone(data.QUATERNION_W)
        self.assertIsNotNone(data.QUATERNION_X)
        self.assertIsNotNone(data.QUATERNION_Y)
        self.assertIsNotNone(data.QUATERNION_Z)
        self.assertIsNotNone(data.VELOCITY_X)
        self.assertIsNotNone(data.VELOCITY_Y)
        self.assertIsNotNone(data.VELOCITY_Z)
        self.assertIsNotNone(data.ACCEL_X)
        self.assertIsNotNone(data.ACCEL_Y)
        self.assertIsNotNone(data.ACCEL_Z)


class GPSMapWidgetTests(unittest.TestCase):
    def make_tile_test_dir(self, name):
        test_dir = PROJECT_ROOT / "tests" / name
        if test_dir.exists():
            shutil.rmtree(test_dir)
        test_dir.mkdir()
        self.addCleanup(lambda: shutil.rmtree(test_dir, ignore_errors=True))
        return test_dir

    def test_build_draw_circle_js_for_valid_input(self):
        js = GPSMapWidget._build_draw_circle_js(45.5, -73.6, 120.0)

        self.assertEqual(js, "window.drawGpsCircle(45.5, -73.6, 120.0);")

    def test_build_draw_circle_js_rejects_invalid_values(self):
        self.assertIsNone(GPSMapWidget._build_draw_circle_js(95.0, -73.6, 120.0))
        self.assertIsNone(GPSMapWidget._build_draw_circle_js(45.5, -190.0, 120.0))
        self.assertIsNone(GPSMapWidget._build_draw_circle_js(45.5, -73.6, 0.0))

    def test_tile_checker_can_be_disabled_from_config(self):
        widget = GPSMapWidget.__new__(GPSMapWidget)
        widget._log_error = lambda _msg: self.fail("disabled checker should not log missing tiles")

        temp_dir = self.make_tile_test_dir("_tmp_tile_checker_disabled")

        GPSMapWidget._check_specific_tiles(
            widget,
            temp_dir,
            {"enabled": False},
        )

    def test_tile_checker_uses_configured_ranges(self):
        widget = GPSMapWidget.__new__(GPSMapWidget)
        errors = []
        widget._log_error = errors.append

        temp_dir = self.make_tile_test_dir("_tmp_tile_checker_ranges")
        z_dir = temp_dir / "12" / "1141"
        z_dir.mkdir(parents=True)
        (z_dir / "tile.png").write_bytes(b"")

        GPSMapWidget._check_specific_tiles(
            widget,
            temp_dir,
            {
                "enabled": True,
                "ranges": {
                    "12": [1141, 1142],
                },
            },
        )

        self.assertEqual(errors, [])

    def test_invalid_tile_checker_ranges_keep_default_checks(self):
        ranges = GPSMapWidget._tile_check_ranges({"12": ["bad", 1142]})

        self.assertEqual(list(ranges[12]), [1141, 1142])


class PainterStub:
    def __init__(self):
        self.calls = []

    def save(self):
        self.calls.append("save")

    def restore(self):
        self.calls.append("restore")


class PayloadVisualizationCanvasTests(unittest.TestCase):
    def test_draw_payload_body_uses_small_uav_shape_helpers(self):
        canvas = PayloadVisualizationCanvas.__new__(PayloadVisualizationCanvas)
        painter = PainterStub()
        rect = object()
        calls = []

        def record(name):
            def _record(*args):
                calls.append(name)

            return _record

        def reject_quadcopter_helper(*args):
            raise AssertionError("quadcopter helpers should not draw the CANSAT payload")

        def reject_overdrawn_helper(*args):
            raise AssertionError("over-detailed render helpers should be replaced by a small UAV-like plane")

        expected_order = [
            "shadow",
            "tail",
            "main_wings",
            "fuselage",
            "canopy",
            "nose_marks",
            "highlights",
            "midline",
        ]

        canvas._draw_payload_shadow = record("shadow")
        canvas._draw_payload_tail = record("tail")
        canvas._draw_payload_main_wings = record("main_wings")
        canvas._draw_payload_fuselage = record("fuselage")
        canvas._draw_payload_canopy = record("canopy")
        canvas._draw_payload_nose_marks = record("nose_marks")
        canvas._draw_payload_highlights = record("highlights")
        canvas._draw_midline = record("midline")
        canvas._draw_quadcopter_shadow = reject_quadcopter_helper
        canvas._draw_quadcopter_arms = reject_quadcopter_helper
        canvas._draw_quadcopter_rotors = reject_quadcopter_helper
        canvas._draw_quadcopter_body = reject_quadcopter_helper
        canvas._draw_payload_rear_wing = reject_overdrawn_helper
        canvas._draw_payload_side_face = reject_overdrawn_helper
        canvas._draw_payload_top_shell = reject_overdrawn_helper
        canvas._draw_payload_foreground_wing = reject_overdrawn_helper
        canvas._draw_payload_wing_root_plates = reject_overdrawn_helper
        canvas._draw_payload_side_mechanism = reject_overdrawn_helper
        canvas._draw_payload_top_camera = reject_overdrawn_helper
        canvas._payload_reference_points = lambda _rect: {
            "top_center": "top-center",
            "center": "center",
            "nose": "nose",
        }

        result = canvas._draw_payload_body(painter, rect)

        self.assertEqual(calls, expected_order)
        self.assertEqual(
            result,
            {
                "top_center": "top-center",
                "center": "center",
                "nose": "nose",
            },
        )

    def test_payload_reference_points_anchor_small_uav_shape(self):
        canvas = PayloadVisualizationCanvas.__new__(PayloadVisualizationCanvas)
        rect = object()
        canvas._project = lambda point, _rect: point
        canvas._rotate = lambda point: point

        self.assertTrue(hasattr(PayloadVisualizationCanvas, "_payload_reference_points"))

        points = PayloadVisualizationCanvas._payload_reference_points(canvas, rect)

        self.assertEqual(points["center"], (0.0, 0.0, 0.0))
        self.assertEqual(points["nose"], (0.0, 0.36, 1.86))
        self.assertEqual(points["top_center"], (0.0, 0.52, 1.18))


if __name__ == "__main__":
    unittest.main()
