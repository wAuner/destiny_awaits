import pid as PID
import yaw_controller 
import lowpass
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


wheel_base = 0
steer_ratio = 0
max_lat_accel = 0
max_steer_angle = 0


class Controller(object):
    def __init__(self, parmams):
       

        self.vehicle_mass = parmams['vehicle_mass']
        self.fuel_capacity = parmams['fuel_capacity']
        self.brake_deadband = parmams['brake_deadband']
        self.decel_limit = parmams['decel_limit']
        self.accel_limit = parmams['accel_limit']
        self.wheel_radius = parmams['wheel_radius']
        self.wheel_base = parmams['wheel_base']
        self.steer_ratio = parmams['steer_ratio']
        self.max_lat_accel = parmams['max_lat_accel']
        self.max_steer_angle = parmams['max_steer_angle']

        self.pid_accel = PID.PID(0.3,0.05,0.05, mn = self.decel_limit, mx = self.accel_limit)




        self.yaw_cont = yaw_controller.YawController(self.wheel_base,self.steer_ratio, 0.1, self.max_lat_accel,self.max_steer_angle)

        self.vel_LP = lowpass.LowPassFilter(0.5,0.02)

    def control(self, linear_vel,angular_vel,current_velocity,enabled):
        
        if not enabled:
            self.pid_accel.reset()
            return 0.,0.,0.

        current_velocity = self.vel_LP.filt(current_velocity)

    	diff_vel = linear_vel - current_velocity




    	steer = self.yaw_cont.get_steering(linear_vel,angular_vel,current_velocity)


        brake = 0
        throttle = self.pid_accel.step(diff_vel, 1./50.)

        if linear_vel == 0. and current_velocity < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.1 and diff_vel < 0:
            brake = abs(max(self.decel_limit,diff_vel))* self.vehicle_mass*self.wheel_radius

        return throttle, brake, steer

