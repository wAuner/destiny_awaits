import pid as PID
import yaw_controller 
import lowpass

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


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

        self.pid_accel = PID.PID(0.3,0.005,0.01, mn = self.decel_limit, mx = self.accel_limit)

        self.yaw_cont = yaw_controller.init(wheel_base,steer_ratio, 0, max_lat_accel,max_steer_angle)



        pass

    def control(self, linear_vel,angular_vel,current_velocity,enabled):
        
    	diff_vel = linear_vel - current_velocity

    	speed = self.pid_accel.step(diff_vel, 1/50)

    	steer = self.yaw_cont.get_steering(linear_vel,angular_vel,current_velocity)

        return 1., 0., steer

