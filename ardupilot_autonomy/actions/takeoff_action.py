#!/usr/bin/env python3
"""
Takeoff Action - Simple and clean
Calls existing services: set_guided_mode -> arm -> takeoff
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import time

from ardupilot_autonomy.state_machine import State

class TakeoffAction:
    """Takeoff action handler"""

    def __init__(self, node, mavros, state_machine):
        self.node = node
        self.mavros = mavros
        self.fsm = state_machine

        # Import action type
        from ardupilot_autonomy_msgs.action import Takeoff

        # Create service clients for existing services
        self.guided_mode_client = self.node.create_client(Trigger, '/vehicle/set_guided_mode')
        self.arm_client = self.node.create_client(Trigger, '/vehicle/arm')
        self.takeoff_client = self.node.create_client(Trigger, '/vehicle/takeoff')

        # Create action server
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self.node,
            Takeoff,
            '/vehicle/takeoff_action',
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )

        self.node.get_logger().info('Takeoff action server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal"""
        # Check FSM (DUMMY - not enforced)
        # if not self.fsm.can_takeoff():
        #     self.node.get_logger().error('Takeoff rejected by FSM')
        #     return GoalResponse.REJECT

        # Check busy flag
        if self.node.busy:
            self.node.get_logger().error('Another action is running')
            return GoalResponse.REJECT

        self.node.get_logger().info(f'Takeoff accepted: {goal_request.altitude}m')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation"""
        self.node.get_logger().info('Takeoff cancel requested')
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        """Execute takeoff"""
        self.node.get_logger().info('Starting takeoff...')

        # Get goal (default to 3m if not provided)
        goal = goal_handle.request
        target_altitude = goal.altitude if goal.altitude > 0 else 3.0

        # Create result and feedback
        from ardupilot_autonomy_msgs.action import Takeoff
        result = Takeoff.Result()
        feedback = Takeoff.Feedback()

        # Set busy
        self.node.busy = True

        try:
            # ========================================
            # Step 1: Call set_guided_mode service
            # ========================================
            self.node.get_logger().info('Calling /vehicle/set_guided_mode service...')
            self.fsm.set_state(State.GUIDED_PRETAKEOFF)  # DUMMY FSM call

            if not self.guided_mode_client.wait_for_service(timeout_sec=5.0):
                raise Exception('set_guided_mode service not available')

            request = Trigger.Request()
            future = self.guided_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            if not future.result().success:
                raise Exception('Failed to set GUIDED mode')

            time.sleep(1.0)

            # Check cancel
            if goal_handle.is_canceling():
                self.abort()
                goal_handle.canceled(result)
                result.success = False
                result.message = 'Cancelled during mode switch'
                return result

            # ========================================
            # Step 2: Call arm service
            # ========================================
            self.node.get_logger().info('Calling /vehicle/arm service...')
            self.fsm.set_state(State.ARMED)  # DUMMY FSM call

            if not self.arm_client.wait_for_service(timeout_sec=5.0):
                raise Exception('arm service not available')

            request = Trigger.Request()
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            if not future.result().success:
                raise Exception('Failed to arm')

            time.sleep(2.0)

            # Check cancel
            if goal_handle.is_canceling():
                self.abort()
                goal_handle.canceled(result)
                result.success = False
                result.message = 'Cancelled during arming'
                return result

            # ========================================
            # Step 3: Set takeoff altitude parameter and call takeoff service
            # ========================================
            self.node.get_logger().info(f'Setting takeoff altitude to {target_altitude}m and calling service...')
            self.fsm.set_state(State.TAKING_OFF)  # DUMMY FSM call

            # Set parameter for takeoff altitude
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            param = Parameter()
            param.name = 'takeoff_altitude'
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=target_altitude)
            self.node.set_parameters([param])

            if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
                raise Exception('takeoff service not available')

            request = Trigger.Request()
            future = self.takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

            if not future.result().success:
                raise Exception('Failed to takeoff')

            time.sleep(1.0)

            # ========================================
            # Step 4: Monitor altitude
            # ========================================
            rate = self.node.create_rate(10)  # 10Hz

            while rclpy.ok():
                # Check cancel
                if goal_handle.is_canceling():
                    self.abort()
                    goal_handle.canceled(result)
                    result.success = False
                    result.message = 'Cancelled during climb'
                    return result

                # Get altitude
                current_alt = self.mavros.current_alt - self.mavros.home_alt

                # Calculate progress
                progress = min(100.0, (current_alt / target_altitude) * 100.0)

                # Publish feedback
                feedback.current_altitude = current_alt
                feedback.progress = progress
                goal_handle.publish_feedback(feedback)

                self.node.get_logger().info(
                    f'Altitude: {current_alt:.1f}m / {target_altitude}m ({progress:.0f}%)',
                    throttle_duration_sec=1.0
                )

                # Check if reached (90% threshold)
                if current_alt >= target_altitude * 0.9:
                    self.node.get_logger().info(f'✓ Reached {current_alt:.1f}m')
                    break

                rate.sleep()

            # ========================================
            # Success!
            # ========================================
            self.fsm.set_state(State.MC_HOVER)  # DUMMY FSM call

            goal_handle.succeed()
            result.success = True
            result.message = f'Takeoff complete at {current_alt:.1f}m'

            self.node.get_logger().info('🎉 Takeoff succeeded!')
            return result

        except Exception as e:
            # Error handling
            self.node.get_logger().error(f'Takeoff failed: {e}')
            self.abort()

            goal_handle.abort()
            result.success = False
            result.message = f'Error: {str(e)}'
            return result

        finally:
            # Always release busy
            self.node.busy = False

    def abort(self):
        """
        DUMMY abort - switch to LOITER mode
        Later we'll call self.node.abort_action()
        """
        self.node.get_logger().warn('🛑 Aborting - switching to LOITER')
        # self.mavros.set_mode("LOITER")
        self.fsm.set_state(State.MC_HOVER)  # DUMMY FSM call