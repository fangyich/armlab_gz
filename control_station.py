#!/usr/bin/python3
import sys
import cv2
import numpy as np
import time
import signal

from rexarm import (rexarm, gzrexarm, rexarm_com)
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)

import os
os.sys.path.append('dynamixel/') # Path setting
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

os.sys.path.append('gazebo/')
from gzclient import GzSimClient

from ui import Ui_MainWindow
# from kinect import Kinect
from trajectory_planner import TrajectoryPlanner
from state_machine import StateMachine


""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
#pixel Positions of image in GUI
MIN_X = 240
MAX_X = 880
MIN_Y = 40
MAX_Y = 520

""" Serial Port Parameters"""
BAUDRATE   = 1000000
DEVICENAME = "/dev/ttyACM0".encode('utf-8')

"""Threads"""
class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage)

    def __init__(self, kinect, parent=None):
        QThread.__init__(self, parent=parent) 
        self.kinect = kinect

    def run(self):
        while True:
            self.kinect.captureVideoFrame()
            self.kinect.captureDepthFrame()
            rgb_frame = self.kinect.convertFrame()
            depth_frame = self.kinect.convertDepthFrame()
            self.updateFrame.emit(rgb_frame, depth_frame)
            time.sleep(.03)

class LogicThread(QThread):   
    def __init__(self, state_machine, parent=None):
        QThread.__init__(self, parent=parent) 
        self.sm = state_machine

    def run(self):
        while True:    
            self.sm.run()
            time.sleep(0.05)

class DisplayThread(QThread):
    updateStatusMessage = pyqtSignal(str)
    updateJointReadout = pyqtSignal(list, list)
    updateEndEffectorReadout = pyqtSignal(list, list)

    def __init__(self, state_machine, real_rexarm, gzrexarm, mode, parent=None):
        QThread.__init__(self, parent=parent)
        self.sm = state_machine 
        self.mode = mode
        self.real_rexarm = real_rexarm
        self.gzrexarm = gzrexarm

    def run(self):
        while True:
            self.updateStatusMessage.emit(self.sm.status_message)
            self.updateStatusMessage.emit(self.mode)
            if self.mode == 'SIM':
                self.updateJointReadout.emit(self.gzrexarm.joint_angles_fb, [])    
                self.updateEndEffectorReadout.emit(self.gzrexarm.wrist_pose, [])
            elif self.mode == 'SIM_ATTACHED':
                self.updateJointReadout.emit(self.gzrexarm.joint_angles_fb, 
                                             self.real_rexarm.joint_angles_fb)    
                self.updateEndEffectorReadout.emit(self.gzrexarm.wrist_pose, 
                                                   self.real_rexarm.get_wrist_pose())
            elif self.mode == 'ATTACHED':
                self.updateJointReadout.emit([], self.real_rexarm.joint_angles_fb)    
                self.updateEndEffectorReadout.emit([], self.real_rexarm.get_wrist_pose())
            
            time.sleep(0.1)
    
"""GUI Class"""
class Gui(QMainWindow):
    """ 
    Main GUI Class
    contains the main function and interfaces between 
    the GUI and functions
    """
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        """Set the running mode to simulation"""
        self.mode = 'SIM'

        """ Set GUI to track mouse """
        # QWidget.setMouseTracking(self, True)

        """
        Dynamixel bus
        TODO: add other motors here as needed with their correct address"""
        self.dxlbus = None 
        # port_num = self.dxlbus.port()
        base = DXL_MX(None, 1)
        shld = DXL_MX(None, 2)
        elbw = DXL_MX(None, 3)
        wrst = DXL_AX(None, 4)
        # wrst2 = DXL_AX(5)
        self.dxl = [base, shld, elbw, wrst]

        """Objects Using Other Classes"""
        self.gzclient = GzSimClient(8081)
        self.gzrexarm = gzrexarm.GzRexarm(self.gzclient)
        self.real_rexarm = rexarm.Rexarm((base, shld, elbw, wrst), 0)
        self.rexarm = rexarm_com.RexarmCom(self.gzrexarm, self.real_rexarm, self.mode)
        
        # self.kinect = Kinect()
        self.tp = TrajectoryPlanner(self.rexarm)
        # self.sm = StateMachine(self.rexarm, self.tp, self.kinect)
        self.sm = StateMachine(self.rexarm, self.tp, None)

        """ 
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(self.sm.set_next_state, "calibrate"))
        self.ui.btnClearTrj.clicked.connect(self.clear_trj)
        self.ui.btnAddCube.clicked.connect(self.add_cube)
        
        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip2.valueChanged.connect(self.sliderChange)
        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)

        # self.ui.sldrMaxTorque.valueChanged.connect(self.sliderTqSpChange)
        # self.ui.sldrSpeed.valueChanged.connect(self.sliderTqSpChange)

        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        self.ui.chk_simulation.stateChanged.connect(self.simulationChk)
        self.ui.chk_attached.stateChanged.connect(self.attachChk)
        self.ui.chk_showtrj.stateChanged.connect(self.trjChk)
        
        self.ui.rdoutStatus.setText("Waiting for input")

        """Initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """Lock torque and speed control soider in simulation"""
        # self.lock_tqspslider()

        """Initalize the simulation"""
        self.gzclient.start()
        self.gzrexarm.initialize()

        """Setup Threads"""
        # self.videoThread = VideoThread(self.kinect)
        # self.videoThread.updateFrame.connect(self.setImage)        
        # self.videoThread.start()
        
        self.logicThread = LogicThread(self.sm)
        self.logicThread.start()
        
        self.displayThread = DisplayThread(self.sm, self.real_rexarm, self.gzrexarm, self.mode)
        self.displayThread.updateJointReadout.connect(self.updateJointReadout)
        self.displayThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
        self.displayThread.updateStatusMessage.connect(self.updateStatusMessage)
        self.displayThread.start()
        
        """ 
        Setup Timer 
        this runs the trackMouse function every 50ms
        """
        # self._timer = QTimer(self)
        # self._timer.timeout.connect(self.trackMouse)
        # self._timer.start(50)

    """ Slots attach callback functions to signals emitted from threads"""
    @pyqtSlot(QImage, QImage)
    def setImage(self, rgb_image, depth_image):
        if(self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if(self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))

    @pyqtSlot(list, list)
    def updateJointReadout(self, gzjoints, joints):
        if self.ui.chk_simulation.checkState() == Qt.Checked and len(gzjoints) == 4:
            self.ui.GzrdoutBaseJC.setText(str("%+.2f" % (gzjoints[0]*R2D)))
            self.ui.GzrdoutShoulderJC.setText(str("%+.2f" % (gzjoints[1]*R2D)))
            self.ui.GzrdoutElbowJC.setText(str("%+.2f" % (gzjoints[2]*R2D)))
            self.ui.GzrdoutWristJC.setText(str("%+.2f" % (gzjoints[3]*R2D)))
        if self.ui.chk_attached.checkState() == Qt.Checked and len(joints) >= 4:
            self.ui.rdoutBaseJC.setText(str("%+.2f" % (joints[0]*R2D)))
            self.ui.rdoutShoulderJC.setText(str("%+.2f" % ((joints[1]*R2D)+90.0)))
            self.ui.rdoutElbowJC.setText(str("%+.2f" % (joints[2]*R2D)))
            self.ui.rdoutWristJC.setText(str("%+.2f" % (joints[3]*R2D)))
            self.ui.rdoutWrist2JC.setText(str("%+.2f" % (joints[4]*R2D)))
            if len(joints) > 5:
                self.ui.rdoutWrist3JC.setText(str("%+.2f" % (joints[5]*R2D)))
            else:
                self.ui.rdoutWrist3JC.setText(str("N.A."))

    @pyqtSlot(list, list)
    def updateEndEffectorReadout(self, gzpose, pose):
        if self.ui.chk_simulation.checkState() == Qt.Checked:
            self.ui.GzEEX.setText(str("%+.2f" % (gzpose[0])))
            self.ui.GzEEY.setText(str("%+.2f" % (gzpose[1])))
            self.ui.GzEEZ.setText(str("%+.2f" % (gzpose[2])))
        if self.ui.chk_attached.checkState() == Qt.Checked:
            self.ui.rdoutX.setText(str("%+.2f" % (pos[0])))
            self.ui.rdoutY.setText(str("%+.2f" % (pos[1])))
            self.ui.rdoutZ.setText(str("%+.2f" % (pos[2])))
            self.ui.rdoutT.setText(str("%+.2f" % (pos[3])))
            self.ui.rdoutG.setText(str("%+.2f" % (pos[4])))
            self.ui.rdoutP.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)


    """ Other callback functions attached to GUI elements"""
    def estop(self):
        self.rexarm.set_estop(True)
        self.sm.set_next_state("estop")
       
    def clear_trj(self):
        """Clear the end-effector trajectory marker"""
        msg = '3,1\n'
        self.gzclient.send_msg(msg)

    def add_cube(self):
        """Add a cube to the workspace in Gazebo"""
        msg = '4,1\n'
        self.gzclient.send_msg(msg)

    def reset_sliders(self):
        self.ui.sldrBase.setValue(0)
        self.ui.sldrShoulder.setValue(0)
        self.ui.sldrElbow.setValue(0)
        self.ui.sldrWrist.setValue(0)
        self.ui.sldrGrip1.setValue(0)
        self.ui.sldrGrip2.setValue(0)
        self.ui.sldrMaxTorque.setValue(50)
        self.ui.sldrSpeed.setValue(25)

#    def lock_tqspslider(self):
#        self.ui.sldrMaxTorque.setEnabled(False)
#        self.ui.sldrSpeed.setEnabled(False)

#    def unlock_tqspslider(self):
#        self.ui.sldrMaxTorque.setEnabled(True)
#        self.ui.sldrSpeed.setEnabled(True)

#    def sliderTqSpChange(self):
#        self.rexarm.set_torque_limits([self.ui.sldrMaxTorque.value()/100.0]*
#                                       self.rexarm.num_joints(), update_now=False)
#        self.rexarm.set_speeds_normalized_global(self.ui.sldrSpeed.value()/100.0, 
#                                                 update_now=False)
#        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
#        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        
    def sliderChange(self):
        """ 
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))
        self.ui.rdoutGrip2.setText(str(self.ui.sldrGrip2.value()))

        joint_positions = np.array([self.ui.sldrBase.value()*D2R,
                                    self.ui.sldrShoulder.value()*D2R,
                                    self.ui.sldrElbow.value()*D2R,
                                    self.ui.sldrWrist.value()*D2R,
                                    self.ui.sldrGrip1.value()*D2R,
                                    self.ui.sldrGrip2.value()*D2R])

        self.rexarm.set_positions(joint_positions, update_now=False)
        
        self.rexarm.set_torque_limits([self.ui.sldrMaxTorque.value()/100.0]*
                                       self.rexarm.num_joints(), update_now=False)
        self.rexarm.set_speeds_normalized_global(self.ui.sldrSpeed.value()/100.0, 
                                                 update_now=False)
        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        
    def directControlChk(self, state):
        if state == Qt.Checked:
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)

    def simulationChk(self, state):
        if state == Qt.Checked:
            # Initialized the gzrexarm
            if self.gzclient.is_running() is False:
                self.gzclient.start()
            # Update the running mode
            if self.ui.chk_attached.checkState() == Qt.Checked:
                self.mode = 'SIM_ATTACHED'
            else:
                self.mode = 'SIM'
            # self.lock_tqspslider()
        else:
            if self.gzclient.is_running() is True:
                self.gzclient.stop()
            if self.ui.chk_attached.checkState() == Qt.Checked:
                self.mode = 'ATTACHED'
            else:
                self.mode = 'DETACHED'
            # self.unlock_tqspslider()
        self.rexarm.set_mode(self.mode)
        self.rexarm.initialize()
        self.reset_sliders()

    def attachChk(self, state):
        if state == Qt.Checked:
            if self.dxlbus is None:
                self.dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
            port_num = self.dxl_bus.port()
            for joint in self.dxl:
                joint.set_port(port_num)
                joint.set_mode(2)
            if self.ui.chk_simulation.checkState() == Qt.Checked:
                self.mode = 'SIM_ATTACHED'
            else:
                self.mode = 'ATTACHED'
            # self.unlock_tqspslider()
        else:
            if self.dxlbus is not None:
                self.dxlbus.close()
                self.dxlbus = None
            if self.ui.chk_simulation.checkState() == Qt.Checked:
                self.mode = 'SIM'
            else:
                self.mode = 'DETACHED'
            # self.lock_tqspslider()
        self.rexarm.set_mode(self.mode)
        self.rexarm.initalize()
        self.reset_sliders()

    def trjChk(self, state):
        if state == Qt.Checked:
            msg = '3,2\n'
        else:
            msg = '3,3\n'
        self.gzclient.send_msg(msg)

    def trackMouse(self):
        """ 
        Mouse position presentation in GUI
        TODO: after implementing workspace calibration 
        display the world coordinates the mouse points to 
        in the RGB video image.
        """
        x = QWidget.mapFromGlobal(self,QCursor.pos()).x()
        y = QWidget.mapFromGlobal(self,QCursor.pos()).y()
        if ((x < MIN_X) or (x >= MAX_X) or (y < MIN_Y) or (y >= MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutMouseWorld.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            if(self.kinect.currentDepthFrame.any() != 0):
                z = self.kinect.currentDepthFrame[y][x]
                self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (x,y,z))
                self.ui.rdoutMouseWorld.setText("(-,-,-)")

    def mousePressEvent(self, QMouseEvent):
        """ 
        Function used to record mouse click positions for calibration 
        """
 
        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

        """ Change coordinates to image axis """
        self.kinect.last_click[0] = x - MIN_X 
        self.kinect.last_click[1] = y - MIN_Y
        self.kinect.new_click = True
        print(self.kinect.last_click)


"""signal handler"""
def sigint_handler(*args):
    """Handler for the SIGINT signal (CTRL-C)"""
    sys.stderr.write('\rTerminate the application...\n')
    QApplication.quit()

"""main function"""
def main():
    signal.signal(signal.SIGINT, sigint_handler)
    app = QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())
 
if __name__ == '__main__':
    main()
