#!/usr/bin/env python3
"""
Vehicle Interface - Main orchestrator node
Provides ROS 2 services for basic flight operations
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionServer
import time

from ardupilot_autonomy_msgs.action import Takeoff
from ardupilot_autonomy.actions.takeoff_action import TakeoffAction


class VehicleInterface(Node):
    """
    Main orchestrator for MAVROS autonomy.
    Exposes simple services for arm, takeoff, goto, land, RTL.
    """
    
    def __init__(self):
        super().__init__('vehicle_interface')
        
        # Import here to avoid circular dependency
        from ardupilot_autonomy.mavros_interface import MavrosInterface
        # from ardupilot_autonomy.dds_interface import DDSInterface
        from ardupilot_autonomy.state_machine import StateMachine, State
        
        self.State = State
        
        # Initialize components
        self.mavros = MavrosInterface(self)
        # self.mavros = DDSInterface(self)
        self.state_machine = StateMachine()

        # Initialize action handlers
        self.takeoff_action_handler = TakeoffAction(self, self.mavros, self.state_machine)

        # Create action server (delegates to handler)
        self._takeoff_action_server = ActionServer(
            self,
            Takeoff,
            '/vehicle/takeoff_action',
            self.takeoff_action_handler.execute
        )
        
        # Simple busy flag (no locks yet)
        self.busy = False
        
        # Declare parameters
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('goto_north', 0.0)
        self.declare_parameter('goto_east', 0.0)
        self.declare_parameter('goto_up', 0.0)
        self.declare_parameter('goto_yaw', 0.0)

        # Velocity control parameters
        self.declare_parameter('vel_x', 0.0)
        self.declare_parameter('vel_y', 0.0)
        self.declare_parameter('vel_z', 0.0)
        self.declare_parameter('vel_yaw_rate', 0.0)

        # Acceleration control parameters
        self.declare_parameter('accel_x', 0.0)
        self.declare_parameter('accel_y', 0.0)
        self.declare_parameter('accel_z', 0.0)
        
        # Create services
        self.create_service(Trigger, '/vehicle/set_guided_mode', self.set_guided_mode_callback)
        self.create_service(Trigger, '/vehicle/arm', self.arm_callback)
        self.create_service(Trigger, '/vehicle/disarm', self.disarm_callback)
        self.create_service(Trigger, '/vehicle/takeoff', self.takeoff_callback)
        self.create_service(Trigger, '/vehicle/goto_position', self.goto_position_callback)
        self.create_service(Trigger, '/vehicle/goto_neu', self.goto_neu_callback)
        self.create_service(Trigger, '/vehicle/land', self.land_callback)
        self.create_service(Trigger, '/vehicle/rtl', self.rtl_callback)
        self.create_service(Trigger, '/vehicle/velocity_start', self.velocity_start_callback)
        self.create_service(Trigger, '/vehicle/velocity_stop', self.velocity_stop_callback)
        self.create_service(Trigger, '/vehicle/accel_start', self.accel_start_callback)
        self.create_service(Trigger, '/vehicle/accel_stop', self.accel_stop_callback)

        # Create action server (delegates to handler)
        self._takeoff_action_server = rclpy.action.ActionServer(
        self,
        Takeoff,
        '/vehicle/takeoff_action',
        self.takeoff_action_handler.execute
        )
        
        # Status timer
        self.status_timer = self.create_timer(3.0, self.status_callback)
        
        self.get_logger().info('Vehicle Interface initialized')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /vehicle/arm')
        self.get_logger().info('  /vehicle/disarm')
        self.get_logger().info('  /vehicle/takeoff')
        self.get_logger().info('  /vehicle/goto_position')
        self.get_logger().info('  /vehicle/goto_neu')
        self.get_logger().info('  /vehicle/land')
        self.get_logger().info('  /vehicle/rtl')
        self.get_logger().info('  /vehicle/velocity_start')
        self.get_logger().info('  /vehicle/velocity_stop')
        self.get_logger().info('  /vehicle/accel_start')
        self.get_logger().info('  /vehicle/accel_stop')

        self.get_logger().info('  /vehicle/takeoff_action')

    def status_callback(self):
        """Periodic status printout"""
        self.get_logger().info(
            f'State: {self.state_machine.get_state().name} | '
            f'Armed: {self.mavros.armed} | '
            f'Mode: {self.mavros.mode} | '
            f'Pos: ({self.mavros.current_lat:.6f}, {self.mavros.current_lon:.6f}, {self.mavros.current_alt:.1f}m)'
        )

    def set_guided_mode_callback(self, request, response):
        """Set GUIDED mode"""
        if self.busy:
            response.success = False
            response.message = 'Busy'
            return response

        # if not self.state_machine.can_set_guided():  # FSM check
        #     response.success = False
        #     response.message = f'Cannot set GUIDED from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            self.mavros.set_mode("GUIDED")
            response.success = True
            response.message = 'GUIDED mode set'
        finally:
            self.busy = False
        return response

    def arm_callback(self, request, response):
        """Service to arm the vehicle"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_arm():
        #     response.success = False
        #     response.message = f'Cannot arm from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            self.mavros.arm()

            # self.state_machine.set_state(self.State.ARMED)
            response.success = True
            response.message = 'Arm command sent'
        finally:
            self.busy = False

        return response
    
    def disarm_callback(self, request, response):
        """Service to disarm the vehicle"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response
        
        self.busy = True
        
        try:
            if not self.mavros.disarm():
                response.success = False
                response.message = 'Disarm failed'
                return response
            
            # self.state_machine.set_state(self.State.IDLE)
            response.success = True
            response.message = 'Disarmed successfully'
            
        finally:
            self.busy = False
        
        return response
    
    def takeoff_callback(self, request, response):
        """Service to takeoff"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response
        
        # if not self.state_machine.can_takeoff():
        #     response.success = False
        #     response.message = f'Cannot takeoff from state: {self.state_machine.get_state().name}'
        #     return response
        
        self.busy = True
        
        try:
            altitude = self.get_parameter('takeoff_altitude').value
            
            if not self.mavros.takeoff(altitude):
                response.success = False
                response.message = 'Takeoff command failed'
                return response
            
            # self.state_machine.set_state(self.State.AIRBORNE)
            response.success = True
            response.message = f'Takeoff to {altitude}m commanded'
            
        finally:
            self.busy = False
        
        return response
    
    def goto_position_callback(self, request, response):
        """Service to goto GPS position"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response
        
        # if not self.state_machine.can_goto():
        #     response.success = False
        #     response.message = f'Cannot goto from state: {self.state_machine.get_state().name}'
        #     return response
        
        self.busy = True
        
        try:
            north = self.get_parameter('goto_north').value
            east = self.get_parameter('goto_east').value
            alt = self.get_parameter('goto_up').value
            yaw = self.get_parameter('goto_yaw').value
            
            if not self.mavros.goto_position(north, east, alt, yaw):
                response.success = False
                response.message = 'Goto position failed'
                return response
            
            response.success = True
            response.message = f'Goto position commanded: ({north:.6f}, {east:.6f}, {alt:.1f}m, {yaw})'
            
        finally:
            self.busy = False
        
        return response


    def goto_neu_callback(self, request, response):
        """Service to goto NED position"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response
        
        # if not self.state_machine.can_goto():
        #     response.success = False
        #     response.message = f'Cannot goto from state: {self.state_machine.get_state().name}'
        #     return response
        
        self.busy = True
        
        try:
            north = self.get_parameter('goto_north').value
            east = self.get_parameter('goto_east').value
            up = self.get_parameter('goto_up').value
            yaw = self.get_parameter('goto_yaw').value
            
            if not self.mavros.goto_neu(north, east, up, yaw):
                response.success = False
                response.message = 'Goto NED failed'
                return response
            
            response.success = True
            response.message = f'Goto NED commanded: N={north:.1f}, E={east:.1f}, D={up:.1f}m'
            
        finally:
            self.busy = False
        
        return response

    def velocity_start_callback(self, request, response):
        """Service to start velocity control"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_velocity_start():
        #     response.success = False
        #     response.message = f'Cannot start velocity from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            vx = self.get_parameter('vel_x').value
            vy = self.get_parameter('vel_y').value
            vz = self.get_parameter('vel_z').value
            yaw_rate = self.get_parameter('vel_yaw_rate').value

            self.mavros.set_velocity(vx, vy, vz, yaw_rate)

            response.success = True
            response.message = f'Velocity started: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}'
        finally:
            self.busy = False

        return response

    def velocity_stop_callback(self, request, response):
        """Service to stop velocity control"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_velocity_stop():
        #     response.success = False
        #     response.message = f'Cannot stop velocity from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:

            # # Switch to LOITER mode (ArduPilot holds position)
            # self.mavros.set_mode("LOITER")

            # Wait for mode switch
            # time.sleep(5.0)

            # stop velocity
            self.mavros.stop_velocity()



            response.success = True
            response.message = 'Velocity control stopped'
        finally:
            self.busy = False

        return response

    def accel_start_callback(self, request, response):
        """Service to start acceleration control"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_accel_start():
        #     response.success = False
        #     response.message = f'Cannot start accel from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            ax = self.get_parameter('accel_x').value
            ay = self.get_parameter('accel_y').value
            az = self.get_parameter('accel_z').value

            self.mavros.set_acceleration(ax, ay, az)

            response.success = True
            response.message = f'Acceleration started: ax={ax}, ay={ay}, az={az}'
        finally:
            self.busy = False

        return response

    def accel_stop_callback(self, request, response):
        """Service to stop acceleration control"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_accel_stop():
        #     response.success = False
        #     response.message = f'Cannot stop accel from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            self.mavros.stop_acceleration()

            response.success = True
            response.message = 'Acceleration control stopped'
        finally:
            self.busy = False

        return response

    
    def land_callback(self, request, response):
        """Service to land"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response
        
        # if not self.state_machine.can_land():
        #     response.success = False
        #     response.message = f'Cannot land from state: {self.state_machine.get_state().name}'
        #     return response
        
        self.busy = True
        
        try:
            if not self.mavros.land():
                response.success = False
                response.message = 'Land command failed'
                return response
            
            # self.state_machine.set_state(self.State.LANDED)
            response.success = True
            response.message = 'Land commanded'
            
        finally:
            self.busy = False
        
        return response

    def rtl_callback(self, request, response):
        """Service to return to launch"""
        if self.busy:
            response.success = False
            response.message = 'Busy - another command in progress'
            return response

        # FSM check
        # if not self.state_machine.can_land():  # RTL has same requirements as land
        #     response.success = False
        #     response.message = f'Cannot RTL from state: {self.state_machine.get_state().name}'
        #     return response

        self.busy = True
        try:
            self.mavros.set_mode("RTL")

            # self.state_machine.set_state(self.State.LANDED)
            response.success = True
            response.message = 'RTL command sent'
        finally:
            self.busy = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vehicle interface')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
