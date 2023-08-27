from quadControl import *
import numpy as np
import time

class move_rect():
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
        print('Desired pos = ', self.x_desired,self.y_desired,self.z_desired)

        # desired velocity
        self.xvel_desired = 0
        self.yvel_desired = 0
        self.zvel_desired = 0

        #start timer
        self.timer = time.time()
        self.target_time=0

        self.abort = 0
    
    def rect(self):
        # Hover
        self.set_target_time(7)
        print("Changing Position")
        # Move to first corner
        self.x_desired=self.x_desired+0.64 # 0.64 is the scaled value with the camera corresponding to 2m on ground 
        self.set_target_time(12)
        # Move to second corner
        self.y_desired=self.y_desired+0.33 # 0.33 is the scaled value with the camera corresponding to 1m on ground
        print("Changing Position")
        self.set_target_time(17)
        # Move to third corner
        self.x_desired=self.x_desired-0.64
        print("Changing Position")
        self.set_target_time(22)
        # Move to fourth corner
        self.y_desired=self.y_desired-0.33
        print("Changing Position")
        self.set_target_time(27)
        print("Hover")
        self.set_target_time(30)

    def path(self):
        self.control.posControl(self.x_desired,self.y_desired,self.xvel_desired,self.yvel_desired,self.abort)

    def set_target_time(self,target_time=10):
  
        while (time.time() - self.timer) < target_time: 

            try:            
                self.path()            
                
            except KeyboardInterrupt:
                # emergency landing
                self.abort = 1                            
                return

if __name__ == '__main__':

    handler = move_rect()
    handler.rect()
    handler.control.posControl(0,0,0,0,1)

    
