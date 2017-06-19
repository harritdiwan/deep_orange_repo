import socket
import atexit
import json
import math
from controller.motion.brain import brain


class carsim(brain):
    def __init__(self,map,car):
        brain.__init__(self,map,car)
        self.sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_DGRAM)  # UDP
        self.sock.bind(("0.0.0.0", 25000))
        self.sock.setblocking(0)
        atexit.register(self.cleanup)

    def cleanup(self):
        if self.sock:
            self.sock.close()

    def setinput(self):
        try:
            data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
            jd = json.loads(data)
            x = jd['position']['x']
            y = self.map.transform_y_back(jd['position']['y'])
            self.car.pose = -math.radians(jd['orientation']['z'])
            self.car.center = (x, y)
            self.car.speed = jd['velocity']['x'] * 3.6
        except socket.error:
            '''no data received..'''
            '''simulate with the model?'''
            pass
        return True
