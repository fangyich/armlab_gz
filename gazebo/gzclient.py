import subprocess
import time
import socket


class GzSimClient():

    def __init__(self, service_port=8081, start=False):
        self._service_port = str(service_port)
        self._server_address = ('localhost', service_port)
        self._subprocess = None
        self._is_running = start
        if(start):
            self.start()

    def __del__(self):
        if self._subprocess is not None:
            self._subprocess.kill()
            print("gzclient subprocess killed.")

    def is_running(self):
        return self._is_running

    def start(self):
        """ Launch the subprocess to interface with the Gazebo server"""
        self._subprocess = subprocess.Popen(["./gazebo/build/c_client",
                                             self._service_port])
        time.sleep(1)
        self._is_running = True
        print("gzclient launched")

    def stop(self):
        self._subprocess.kill()
        self._subprocess = None
        self._is_running = False
        print("gzclient closed")

    def send_msg(self, msg, response=False):
        """Send the request through TCP/IP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(self._server_address)
        sock.sendall(msg.encode('utf-8'))

        if response is True:
            data = sock.recv(1024)
            try:
                res = data.decode('utf-8')
            except UnicodeDecodeError:
                print("[ERROR] Unable to parse the message")
                res = ""
        else:
            res = ""
        return res

    def get_cube_info(self):
        """Get the pose of the cubes"""
        msg = '5,\n'
        info = self.send_msg(msg, response=True)

        cube_list = []
        cubes = info.split('\n')
        if cubes:
            for c in cubes:
                pose = list(map(float, c.split(',')[1:]))
                if pose:
                    cube_list.append(pose)
