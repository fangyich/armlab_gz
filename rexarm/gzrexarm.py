import numpy as np
import time
from rexarm.baseclass import BaseRexarm


class GzRexarm(BaseRexarm):
    
    def __init__(self, gzclient, gripper=None):
        self.init_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.num_joints = len(self.init_pose)  # num of joints are fixed to 4
        self.gzclient = gzclient
        self.gripper = gripper
        self.gripper_open_pos = np.deg2rad(-60.0)
        self.gripper_closed_pos = np.deg2rad(30.0)
        self.gripper_state = False
        self.estop = False

        """ Physical limits of the Rexarm """
        self.angle_limits = np.deg2rad(np.array([
                                       [-180, 179.99],
                                       [-180, 179.99],
                                       [-180, 179.99],
                                       [-180, 179.99],
                                       [-180, 179.99],
                                       [-180, 179.99]], dtype=np.float))

        """ Commanded Values """
        self.position = [0.0] * self.num_joints     # radians
        # self.speed = [1.0] * self.num_joints        # 0 to 1
        
        # 0 to 1 "effort_limit(%)" in gazebo
        # self.max_torque = [1.0] * self.num_joints   
        
        """ Feedback Values """
        self.wrist_pose = [0.0, 0.0, 0.0] # (m)
        self.joint_angles_fb = [0.0] * self.num_joints  # radians
        # self.speed_fb = [0.0] * self.num_joints         # 0 to 1   

    def initialize(self):
        self.set_positions(self.init_pose)
        # joint.enable_torque()
        # joint.set_torque_limit(0.5)
        # joint.set_speed(0.25)
        # if(self.gripper != 0):
        #     self.gripper.set_torque_limit(1.0)
        #     self.gripper.set_speed(0.8)
        #     self.close_gripper()

    def open_gripper(self):
        """ TODO """
        self.gripper_state = False

    def close_gripper(self):
        """ TODO """
        self.gripper_state = True

    def toggle_gripper(self):
        """ TODO """
        pass

    def set_positions(self, joint_angles, update_now=True):
        """Take positions in degrees from -PI to PI"""
        self.clamp(joint_angles)
        for i in range(self.num_joints):
            self.position[i] = joint_angles[i]
        if update_now:
            # Creates the joint command messsage
            msg = '1,'
            for idx, j in enumerate(joint_angles):
                msg += str(j)
                if idx+1 == len(joint_angles):
                    msg += '\n'
                else:
                    msg += ','
            # Sends the command
            self.gzclient.send_msg(msg)

    def send_commands(self):
        self.set_positions(self.position)

    def get_positions(self):
        """Get the joints angles"""
        # Sends the request
        msg = '2,\n'
        pose_msg = self.gzclient.send_msg(msg, response=True)
        # Parse the message
        pose_msg = pose_msg.split(',')
        if len(pose_msg) == 7:
            try:
                pose = list(map(float, pose_msg))
            except ValueError:
                print("Bad string skip.")
                return self.joint_angles_fb
            self.wrist_pose = pose[0:3]
            self.joint_angles_fb = pose[3:]
        return self.joint_angles_fb

    def get_feedback(self):
        self.get_positions()
        # self.get_speeds()
        # self.get_loads()
        # self.get_temps()
        # self.get_moving_status()

    def pause(self, secs):
        super(GzRexarm, self).pause(secs)

    def clamp(self, joint_angles):
        """Limit the joint angles"""
        for i, angle in enumerate(joint_angles):
            if angle < self.angle_limits[i][0]:
                angle = self.angle_limits[i][0]
            elif angle > self.angle_limits[i][1]:
                angle = self.angle_limits[i][1]

    def get_wrist_pose(self):
        """Get the end-effector pose"""
        return self.wrist_pos
