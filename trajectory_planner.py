import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""
	
class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints()
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    # def set_initial_wp(self):
    def set_initial_wp(self, waypoint):
        self.initial_wp = waypoint

    def set_final_wp(self, waypoint):
        self.final_wp = waypoint
        
    def go(self, max_speed = 2.5):
        Theta_t = 2
        profile = map(self.generate_cubic_spline, self.initial_wp, 
                  self.final_wp, [Theta_t]*4)
        pos_profile = list(zip(*profile))
        
        count = len(pos_profile)
        for i in range(0, count):
            next_p = pos_profile[i]
            self.rexarm.set_positions([next_p[0], next_p[1], next_p[2], next_p[3]])
            time.sleep(self.dt)

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        M = np.matrix([[1,0,0,0],[0,1,0,0],[1,T,T*T,T**3],[0,1,2*T,3*T*T]])
        b = np.matrix([[initial_wp], [0], [final_wp], [0]])
        coef = np.linalg.inv(M) * b
        pos = [coef[0,0]+coef[1,0]*t+coef[2,0]*(t**2)+coef[3,0]*(t**3) for t in np.arange(0,T,self.dt)]
        pos += [final_wp]
 
        return pos

    def execute_plan(self, plan, look_ahead=8):
        pass

