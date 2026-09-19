import os
import sys
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from gui.graph_gui import GraphWindow  # noqa: E402
from plotter.plotters import DynamicPlotterDualAxis  # noqa: E402


def ensure_app():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    return app


class PlotterStub:
    def __init__(self):
        self.calls = []

    def update_plot(self, value):
        self.calls.append(value)


class DynamicPlotterDualAxisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = ensure_app()

    def test_voltage_update_changes_only_voltage_trace(self):
        plotter = DynamicPlotterDualAxis(
            title="Voltage / Current",
            time_window=4,
            x_unit="s",
            left_y_unit="V",
            right_y_unit="mA",
        )

        plotter.update_left_plot(12.4)

        self.assertEqual(plotter.left_y[-1], 12.4)
        self.assertEqual(plotter.right_y[-1], 0.0)

    def test_current_update_changes_only_current_trace(self):
        plotter = DynamicPlotterDualAxis(
            title="Voltage / Current",
            time_window=4,
            x_unit="s",
            left_y_unit="V",
            right_y_unit="mA",
        )

        plotter.update_right_plot(315.0)

        self.assertEqual(plotter.left_y[-1], 0.0)
        self.assertEqual(plotter.right_y[-1], 315.0)

    def test_reset_plot_clears_both_traces_and_time_state(self):
        plotter = DynamicPlotterDualAxis(
            title="Voltage / Current",
            time_window=4,
            x_unit="s",
            left_y_unit="V",
            right_y_unit="mA",
        )
        plotter.update_left_plot(11.8)
        time.sleep(0.01)
        plotter.update_right_plot(280.0)

        plotter.reset_plot()

        self.assertEqual(plotter.last_time, None)
        self.assertTrue((plotter.left_y == 0).all())
        self.assertTrue((plotter.right_y == 0).all())
        self.assertTrue((plotter.x == 0).all())


class GraphWindowRoutingTests(unittest.TestCase):
    def make_window(self):
        window = GraphWindow.__new__(GraphWindow)
        shared_power_plot = PlotterStub()
        accel_xyz_plot = PlotterStub()
        window.plotters = [shared_power_plot, accel_xyz_plot]
        window.graph_title_to_index = {
            "Voltage": 0,
            "Current": 0,
            "AccelXYZ": 1,
        }
        return window, shared_power_plot, accel_xyz_plot

    def test_voltage_and_current_share_same_plotter(self):
        window, shared_power_plot, _ = self.make_window()

        GraphWindow.update_volt_graph(window, 12.1)
        GraphWindow.update_current_graph(window, 305.0)

        self.assertEqual(shared_power_plot.calls, [12.1, 305.0])

    def test_accel_xyz_uses_dedicated_plotter(self):
        window, _, accel_xyz_plot = self.make_window()

        GraphWindow.update_accel_xyz_graph(window, [1.0, 2.0, 3.0])

        self.assertEqual(accel_xyz_plot.calls, [[1.0, 2.0, 3.0]])


if __name__ == "__main__":
    unittest.main()
