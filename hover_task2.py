from quadControl import *
import numpy as np
import time

class hover():
    def __init__(self):
        # calling the control class
        self.control = quadControl()
        
        # initial position (same as the initial position detected by camera)
        self.init_pos_x = self.control.init_pos_x
        self.init_pos_y = self.control.init_pos_y
        self.init_pos_z = self.control.init_pos_z

        # desired position (same as the initial position detected by camera)
        self.x_desired = np.round(self.init_pos_x, 2)
        self.y_desired = np.round(self.init_pos_y, 2)
        self.z_desired = np.round(self.init_pos_z, 2) - 0.3
        print('desired pos = ', self.x_desired,self.y_desired,self.z_desired)

        # desired velocity
        self.xvel_desired = 0
        self.yvel_desired = 0
        self.zvel_desired = 0

        #start timer
        self.timer = time.time()

        self.abort = 0
    
    def path(self):
        self.control.posControl(self.x_desired,self.y_desired,self.xvel_desired,self.yvel_desired,self.abort)

    def set_hover_time(self,hover_time=10):
      
        while (time.time() - self.timer) < hover_time: #hover for 10 sec
            try:            
                self.path()            
                
            except KeyboardInterrupt:
                # emergency landing
                self.abort = 1                            
                return

if __name__ == '__main__':

    hover_control = hover()
    
    hover_control.set_hover_time(10)

    # Land, disarm and disconnect
    hover_control.control.posControl(0,0,0,0,1)

    
