#!/usr/bin/env python3
"""
State Machine - Simple state tracking for immediate goals
Full FSM with 20 states comes later
"""

from enum import Enum


class State(Enum):
    """Simple states for basic flight operations"""
    IDLE = 0        # On ground, disarmed
    ARMED = 1       # Armed, ready for takeoff
    AIRBORNE = 2    # In flight, can accept commands
    LANDED = 3      # Landed, ready to disarm


class StateMachine:
    """Simple state tracking - no complex validation yet"""
    
    def __init__(self):
        self.state = State.IDLE
    
    def get_state(self):
        return self.state
    
    def set_state(self, new_state):
        self.state = new_state
    
    # Simple checks - can expand later
    def can_set_guided(self):
        return self.state == State.IDLE

    def can_arm(self):
        return self.state == State.IDLE
    
    def can_takeoff(self):
        return self.state == State.ARMED
    
    def can_goto(self):
        return self.state == State.AIRBORNE
    
    def can_land(self):
        return self.state == State.AIRBORNE

    def can_velocity_start(self):
        return self.state == State.AIRBORNE

    def can_velocity_stop(self):
        return self.state == State.AIRBORNE

    def can_accel_start(self):
        return self.state == State.AIRBORNE

    def can_accel_stop(self):
        return self.state == State.AIRBORNE
