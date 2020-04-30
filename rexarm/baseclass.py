import abc
import time


class BaseRexarm(metaclass=abc.ABCMeta):
    """This is the abstract class of the rexarm"""

    @abc.abstractmethod
    def __init__(self):
        self.estop = False

    @abc.abstractmethod
    def initialize(self):
        pass

    @abc.abstractmethod
    def open_gripper(self):
        pass

    @abc.abstractmethod
    def close_gripper(self):
        pass

    @abc.abstractmethod
    def toggle_gripper(self):
        pass

    @abc.abstractmethod
    def set_positions(self, joint_angles, update_now=True):
        pass

    @abc.abstractmethod
    def send_commands(self):
        pass

    @abc.abstractmethod
    def get_positions(self):
        pass

    @abc.abstractmethod
    def get_feedback(self):
        pass

    @abc.abstractmethod
    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop is True):
                break

    @abc.abstractmethod
    def clamp(self, joint_angles):
        pass
