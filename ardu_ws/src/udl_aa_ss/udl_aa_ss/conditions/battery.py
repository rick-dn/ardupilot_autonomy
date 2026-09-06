#!/usr/bin/env python3
"""
Battery condition - grades the pack voltage into a band and asks for the
response that band maps to.

The bands and their responses are constants.BATTERY_TIERS; the voltages behind
them are ROS parameters. Neither lives here, so retuning is a number in the yaml
and adding a band is a row in the table - this file only walks it.

safety_monitor never sees a volt. It is handed the response and arbitrates
between it and whatever the other conditions asked for.
"""

from typing import Dict

from udl_aa_ss import constants
from udl_aa_ss.constants import CLEAR, Verdict


def evaluate(telemetry, limits: Dict[str, float], state: Dict) -> Verdict:
    """
    Pack voltage against BATTERY_TIERS. The table is ascending, so the most
    severe band is tested first and the first crossed threshold wins.

    `state` is unused - this is a pure function of the snapshot. The token is
    stubbed at 0 until firing instances are implemented.
    """
    volts = telemetry.battery.voltage
    return next(
        (Verdict(message)
         for level, message in constants.BATTERY_TIERS
         if volts < limits[constants.batt_key(level)]),
        CLEAR,
    )
