from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, car_data):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base=car_data.wheel_base,
                                            steer_ratio=car_data.steer_ratio,
                                            min_speed=car_data.min_speed,
                                            max_lat_accel=car_data.max_lat_accel,
                                            max_steer_angle=car_data.max_steer_angle)

        self.car_data = car_data
        self.pid = PID(kp=5, ki=0.5, kd = 0.5, mn = car_data.decel_limit, mx = car_data.accel_limit)
        self.steer_lpf = LowPassFilter(tau = 3, ts = 1)
        self.throttle_lpf = LowPassFilter(tau = 3, ts = 1)

    def control(self, twist_cmd, current_velocity, time_step):
        # TODO: Change the arg, kwarg list to suit your needs
        linear_vel = abs(twist_cmd.twist.linear.x)
        angular_vel = twist_cmd.twist.angular.z
        velocity_err = linear_vel - current_velocity.twist.linear.x

        steering_val = self.steer_lpf.filt(self.yaw_controller.get_steering(linear_vel, angular_vel, current_velocity.twist.linear.x))

        throttle_val = self.throttle_lpf.filt(self.pid.step(velocity_err, time_step))

        throttle = 0
        brake = 0

        if throttle_val > 0.0:
            throttle = throttle_val
        else:
            if -throttle_val < self.car_data.brake_deadband:
                brake = 0.0
            else:
                brake = -throttle_val * (self.car_data.vehicle_mass + self.car_data.fuel_capacity * GAS_DENSITY) * self.car_data.wheel_radius
        #Return throttle and brake. We will get steering directly in DBW_Node using Yaw.
        return throttle, brake, steering_val

    def reset(self):
        self.pid.reset()
