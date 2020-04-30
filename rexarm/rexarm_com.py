

class RexarmCom():

    def __init__(self, gzrexarm, real_rexarm, mode):
        self.gzrexarm = gzrexarm
        self.real_rexarm = real_rexarm
        self.mode = mode

    def set_mode(self, mode):
        self.mode = mode

    def set_estop(self, value):
        self.gzrexarm.estop = value
        self.real_rexarm.estop = value
    
    def num_joints(self):
        if self.mode == 'SIM':
            num = self.gzrexarm.num_joints
        elif self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            num = self.real_rexarm.num_joints
        else:
            num = 0
        return num

    def initialize(self):
        if self.mode == 'SIM':
            self.gzrexarm.initialize()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.initialize()
            self.real_rexarm.initialize()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.initialize()

    def open_gripper(self):
        if self.mode == 'SIM':
            self.gzrexarm.open_gripper()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.open_gripper()
            self.real_rexarm.open_gripper()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.open_gripper()

    def close_gripper(self):
        if self.mode == 'SIM':
            self.gzrexarm.close_gripper()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.close_gripper()
            self.real_rexarm.close_gripper()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.close_gripper()

    def toggle_gripper(self):
        if self.mode == 'SIM':
            self.gzrexarm.toggle_gripper()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.toggle_gripper()
            self.real_rexarm.toggle_gripper()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.toggle_gripper()

    def set_positions(self, joint_angles, update_now=True):
        if self.mode == 'SIM':
            self.gzrexarm.set_positions(joint_angles, update_now)
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.set_positions(joint_angles, update_now)
            self.real_rexarm.set_positions(joint_angles, update_now)
        elif self.mode == 'ATTACHED':
            self.real_rexarm.set_positions(joint_angles, update_now)

    def set_speeds_normalized_global(self, speed, update_now=True):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.set_speeds_normalized_global(speed, update_now)

    def set_speeds_normalized(self, speeds, update_now=True):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.set_speeds_normalized(speeds, update_now)

    def set_speeds(self, speeds, update_now=True):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.set_speeds(speeds, update_now)

    def set_torque_limits(self, torques, update_now=True):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.set_torque_limits(torques, update_now)

    def send_commands(self):
        if self.mode == 'SIM':
            self.gzrexarm.send_commands()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.send_commands()
            self.real_rexarm.send_commands()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.send_commands()

    def enable_torque(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.enable_torque()

    def disable_torque(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            self.real_rexarm.disable_torque()

    def get_positions(self):
        """Get the position from rexarms """
        if self.mode == 'SIM':
            pos = self.gzrexarm.get_positions()

        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.get_positions()
            pos = self.real_rexarm.get_positions()
        elif self.mode == 'ATTACHED':
            pos = self.real_rexarm.get_positions()
        else:
            pos = []
        return pos

    def get_speeds(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            return self.real_rexarm.get_speeds()

    def get_loads(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            return self.real_rexarm.get_loads()

    def get_temps(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            return self.real_rexarm.get_temps()

    def get_moving_status(self):
        if self.mode == 'SIM_ATTACHED' or self.mode == 'ATTACHED':
            return self.real_rexarm.get_moving_status()

    def get_feedback(self):
        if self.mode == 'SIM':
            self.gzrexarm.get_feedback()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.get_feedback()
            self.real_rexarm.get_feedback()
        elif self.mode == 'ATTACHED':
            self.real_rexarm.get_feedback()

    def pause(self, secs):
        self.gzrexarm.pause()

    def clamp(self, joint_angles):
        if self.mode == 'SIM':
            self.gzrexarm.clamp(joint_angles)
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.clamp(joint_angles)
            self.real_rexarm.clamp(joint_angles)
        elif self.mode == 'ATTACHED':
            self.real_rexarm.clamp(joint_angles)

    def get_wrist_pose(self):
        if self.mode == 'SIM':
            pose = self.gzrexarm.get_wrist_pose()
        elif self.mode == 'SIM_ATTACHED':
            self.gzrexarm.get_wrist_pose()
            pose = self.real_rexarm.get_wrist_pose()
        elif self.mode == 'ATTACHED':
            pose = self.real_rexarm.get_wrist_pose()
        else:
            pose = []
        return pose
