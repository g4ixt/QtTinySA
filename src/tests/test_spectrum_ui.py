"""
Headless checks on the sweep frequency spin boxes declared in spectrum.ui.

These parse the .ui XML directly, so no QApplication or display is needed.

Run from the src directory so that the relative path resolves:
    python -m pytest tests -q
"""

import os
import xml.etree.ElementTree as ET

import pytest

UI_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "modules",
    "spectrum.ui",
)

# All four are wired to valueChanged, which clamps the range and restarts the
# sweep. With keyboard tracking on, that fires on every digit typed.
FREQ_SPIN_BOXES = ("start_freq", "centre_freq", "stop_freq", "span_freq")


@pytest.fixture(scope="module")
def widgets():
    root = ET.parse(UI_PATH).getroot()
    return {
        widget.get("name"): widget
        for widget in root.iter("widget")
        if widget.get("class") == "QDoubleSpinBox"
    }


def properties(widget):
    return {prop.get("name"): prop for prop in widget.findall("property")}


class TestFrequencySpinBoxes:
    @pytest.mark.parametrize("name", FREQ_SPIN_BOXES)
    def test_widget_is_present(self, widgets, name):
        assert name in widgets

    @pytest.mark.parametrize("name", FREQ_SPIN_BOXES)
    def test_keyboard_tracking_is_off(self, widgets, name):
        """Typing must not apply until Enter, focus loss, or a spin button."""
        prop = properties(widgets[name]).get("keyboardTracking")
        assert prop is not None, f"{name} has no keyboardTracking property"
        assert prop.find("bool").text == "false"

    @pytest.mark.parametrize("name", FREQ_SPIN_BOXES)
    def test_still_displays_four_decimals(self, widgets, name):
        # The 4-decimal display is what made partial input unreadable while
        # tracking was on; keep it, since the fix defers the reformat instead.
        assert properties(widgets[name])["decimals"].find("number").text == "4"
