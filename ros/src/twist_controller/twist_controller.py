from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.min_speed = 0.0
        self.yaw_contoller = YawController(self.wheel_base,
                                           self.steer_ratio,
                                           self.min_speed,
                                           self.max_lat_accel,
                                           self.max_steer_angle)

        kp = 0.1
        ki = 0.007
        kd = 0.0
        minth = 0.0
        maxth = 0.2
        self.throttle_contoller = PID(kp, ki, kd, minth, maxth)

        tau = 150
        ts = 75
        self.lpf_velocity = LowPassFilter(tau, ts)
        # self.lpf_steering = LowPassFilter(tau, ts)

        self.t0 = rospy.get_time()

    def control(self, proposed_twist, current_twist, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_contoller.reset()
            return 0.0, 0.0, 0.0

        proposed_linear_velocity = proposed_twist.twist.linear.x
        proposed_angular_velocity = proposed_twist.twist.angular.z
        current_linear_velocity = current_twist.twist.linear.x
        current_angular_velocity = current_twist.twist.angular.z

        current_linear_velocity = self.lpf_velocity.filt(current_linear_velocity)   # Filter current linear velocity
        err_linear_velocity = proposed_linear_velocity - current_linear_velocity

        t1 = rospy.get_time()
        sample_time = t1 - self.t0
        self.t0 = t1

        throttle = self.throttle_contoller.step(err_linear_velocity, sample_time)
        brake = 0.0
        if proposed_linear_velocity == 0.0 and current_linear_velocity < 0.1:
            throttle = 0.0
            brake = 400     # from walkthrough
        elif throttle < 0.1 and err_linear_velocity < 0:
            throttle = 0.0
            decel = max(err_linear_velocity, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        steering = self.yaw_contoller.get_steering(proposed_linear_velocity,
                                                   proposed_angular_velocity,
                                                   current_linear_velocity)

        return throttle, brake, steering
