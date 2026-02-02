#!/usr/bin/env python3
"""
State Machine - FSM with transition matrix and telemetry monitoring
Telemetry updates are DUMMY for now - just placeholders for future
"""

from enum import Enum
import threading


class State(Enum):
    """Flight states - expanded to match JacopoPan's multi-copter states"""
    # Ground states
    IDLE = 0
    STARTED = 1  # Alias for IDLE (JacopoPan compatibility)

    # Pre-flight
    GUIDED_PRETAKEOFF = 2
    ARMED = 3

    # Takeoff
    TAKING_OFF = 4

    # Airborne
    MC_HOVER = 5
    NAVIGATING = 6

    # Orbit states (future)
    MC_ORBIT_PARAM1_SET = 7
    MC_ORBIT_PARAM2_SET = 8
    MC_ORBIT_MISSION_UPLOADED = 9
    MC_ORBIT_MISSION_MODE = 10
    MC_ORBIT_MISSION_STARTED = 11
    MC_ORBIT_TRANSFER = 12
    MC_ORBIT_ROI_SET = 13
    MC_ORBIT_REACHED = 14
    MC_ORBIT = 15

    # Offboard control (velocity/accel)
    OFFBOARD_VELOCITY = 16
    OFFBOARD_ACCELERATION = 17

    # Landing states
    MC_RTL_PARAM_SET = 18
    MC_RTL = 19
    MC_RETURNED_READY_TO_LAND = 20
    MC_LANDING = 21
    LANDED = 22

    # Emergency
    EMERGENCY = 99


class StateMachine:
    """
    State machine with transition matrix and telemetry monitoring.
    Telemetry updates are DUMMY - not actually enforced yet.
    """

    # Valid state transitions (transition matrix)
    VALID_TRANSITIONS = {
        State.IDLE: [State.GUIDED_PRETAKEOFF],
        State.STARTED: [State.GUIDED_PRETAKEOFF],

        State.GUIDED_PRETAKEOFF: [State.ARMED, State.IDLE],
        State.ARMED: [State.TAKING_OFF, State.IDLE],
        State.TAKING_OFF: [State.MC_HOVER, State.EMERGENCY],

        State.MC_HOVER: [
            State.NAVIGATING,
            State.MC_ORBIT_PARAM1_SET,
            State.OFFBOARD_VELOCITY,
            State.OFFBOARD_ACCELERATION,
            State.MC_RTL_PARAM_SET,
            State.MC_LANDING,
            State.EMERGENCY
        ],

        State.NAVIGATING: [
            State.MC_HOVER,
            State.MC_RTL_PARAM_SET,
            State.MC_LANDING,
            State.EMERGENCY
        ],

        # Orbit sequence
        State.MC_ORBIT_PARAM1_SET: [State.MC_ORBIT_PARAM2_SET, State.MC_HOVER],
        State.MC_ORBIT_PARAM2_SET: [State.MC_ORBIT_MISSION_UPLOADED, State.MC_HOVER],
        State.MC_ORBIT_MISSION_UPLOADED: [State.MC_ORBIT_MISSION_MODE, State.MC_HOVER],
        State.MC_ORBIT_MISSION_MODE: [State.MC_ORBIT_MISSION_STARTED, State.MC_HOVER],
        State.MC_ORBIT_MISSION_STARTED: [State.MC_ORBIT_TRANSFER, State.MC_HOVER],
        State.MC_ORBIT_TRANSFER: [State.MC_ORBIT_ROI_SET, State.MC_HOVER],
        State.MC_ORBIT_ROI_SET: [State.MC_ORBIT_REACHED, State.MC_HOVER],
        State.MC_ORBIT_REACHED: [State.MC_ORBIT, State.MC_HOVER],
        State.MC_ORBIT: [
            State.MC_HOVER,
            State.MC_RTL_PARAM_SET,
            State.MC_LANDING,
            State.EMERGENCY
        ],

        # Offboard control
        State.OFFBOARD_VELOCITY: [State.MC_HOVER, State.EMERGENCY],
        State.OFFBOARD_ACCELERATION: [State.MC_HOVER, State.EMERGENCY],

        # Landing sequence
        State.MC_RTL_PARAM_SET: [State.MC_RTL, State.MC_HOVER],
        State.MC_RTL: [State.MC_RETURNED_READY_TO_LAND, State.MC_HOVER],
        State.MC_RETURNED_READY_TO_LAND: [State.MC_LANDING, State.MC_HOVER],
        State.MC_LANDING: [State.LANDED, State.EMERGENCY],
        State.LANDED: [State.IDLE],

        State.EMERGENCY: [State.IDLE],  # Recovery path
    }

    def __init__(self):
        self.state = State.IDLE
        self.lock = threading.RLock()  # Reentrant lock for thread safety

        # Telemetry tracking (DUMMY - not used yet)
        self.telemetry_armed = False
        self.telemetry_mode = ""
        self.telemetry_altitude = 0.0

    def get_state(self):
        """Get current state (thread-safe)"""
        with self.lock:
            return self.state

    def set_state(self, new_state):
        """
        Set new state.
        DUMMY: Validation not enforced yet - always succeeds.
        """
        with self.lock:
            old_state = self.state

            # DUMMY: Just log transition, don't enforce
            if not self._is_valid_transition(new_state):
                # In future, this will raise error or return False
                # For now, just log warning
                print(f"⚠️  Warning: Invalid transition {old_state.name} -> {new_state.name} (allowed anyway)")

            self.state = new_state
            print(f"🔄 FSM: {old_state.name} -> {new_state.name}")

    def _is_valid_transition(self, new_state):
        """Check if transition is valid according to matrix"""
        if self.state not in self.VALID_TRANSITIONS:
            return False
        return new_state in self.VALID_TRANSITIONS[self.state]

    def can_transition(self, new_state):
        """
        Check if transition is allowed.
        DUMMY: Always returns True for now.
        """
        # FUTURE: Will check transition matrix
        # return self._is_valid_transition(new_state)
        return True  # DUMMY - always allow

    # ============================================
    # Telemetry-driven updates (DUMMY - not called yet)
    # ============================================

    def update_from_telemetry(self, armed, mode, altitude):
        """
        DUMMY: Telemetry callback that will update FSM state.
        Not called yet - placeholder for future.

        Args:
            armed: True if motors armed
            mode: Flight mode string (e.g., "GUIDED", "LOITER")
            altitude: Current altitude in meters
        """
        with self.lock:
            # Store telemetry
            self.telemetry_armed = armed
            self.telemetry_mode = mode
            self.telemetry_altitude = altitude

            # FUTURE: State transitions based on telemetry
            # if self.state == State.GUIDED_PRETAKEOFF and armed:
            #     self.set_state(State.ARMED)
            #
            # if self.state == State.TAKING_OFF and altitude > 0.9 * target:
            #     self.set_state(State.MC_HOVER)

            # For now, do nothing
            pass

    def telemetry_callback_armed(self, armed):
        """DUMMY: Will be called when armed state changes"""
        self.telemetry_armed = armed
        # FUTURE: Trigger state transitions
        pass

    def telemetry_callback_mode(self, mode):
        """DUMMY: Will be called when mode changes"""
        self.telemetry_mode = mode
        # FUTURE: Trigger state transitions
        pass

    def telemetry_callback_altitude(self, altitude):
        """DUMMY: Will be called on altitude updates"""
        self.telemetry_altitude = altitude
        # FUTURE: Trigger state transitions
        pass

    # ============================================
    # Precondition checks (for actions to call)
    # ============================================

    def can_arm(self):
        """Can we arm from current state?"""
        return self.state in [State.IDLE, State.STARTED, State.GUIDED_PRETAKEOFF]

    def can_takeoff(self):
        """Can we takeoff from current state?"""
        return self.state in [State.ARMED, State.GUIDED_PRETAKEOFF]

    def can_goto(self):
        """Can we navigate from current state?"""
        return self.state in [State.MC_HOVER, State.NAVIGATING]

    def can_orbit(self):
        """Can we orbit from current state?"""
        return self.state in [State.MC_HOVER, State.MC_ORBIT]

    def can_land(self):
        """Can we land from current state?"""
        return self.state in [
            State.MC_HOVER,
            State.NAVIGATING,
            State.MC_ORBIT,
            State.OFFBOARD_VELOCITY,
            State.OFFBOARD_ACCELERATION
        ]

    def can_velocity_start(self):
        """Can we start velocity control?"""
        return self.state in [State.MC_HOVER, State.OFFBOARD_VELOCITY]

    def can_velocity_stop(self):
        """Can we stop velocity control?"""
        return self.state == State.OFFBOARD_VELOCITY

    def can_accel_start(self):
        """Can we start acceleration control?"""
        return self.state in [State.MC_HOVER, State.OFFBOARD_ACCELERATION]

    def can_accel_stop(self):
        """Can we stop acceleration control?"""
        return self.state == State.OFFBOARD_ACCELERATION

    def can_set_guided(self):
        """Can we switch to GUIDED mode?"""
        return self.state in [State.IDLE, State.STARTED]

    # ============================================
    # Utility
    # ============================================

    def get_state_name(self):
        """Get current state name as string"""
        with self.lock:
            return self.state.name

    def __repr__(self):
        return f"StateMachine(state={self.state.name})"