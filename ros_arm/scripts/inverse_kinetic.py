import math

class inverse_kinetic:
    def __init__(self) -> None:
        self.l1 = 0.15
        self.l2 = 0.2
        self.l3 = 0.2
        self.l4 = 0.03
        self.l5 = 0.06
        self.l6 = 0.02

        self.joint = [0, math.pi/4, math.pi/2, 0, math.pi/4, 0]
        self.gripper = [0, 0]

        self.carmera_midpoint = [360, 640]

    def caculator(self,dx, dy, dz):
        self.joint[0] = math.atan2(dy, dx)
        self.joint[1] = math.atan2(dz, math.sqrt(dx * dx + dy * dy)) - math.atan2(self.l4, self.l3)
        self.joint[2] = math.sqrt(dx * dx + dy * dy + self.z * self.z) - self.l1 - self.l5
        self.joint[3] = math.atan2(self.y, self.x)
        self.joint[4] = math.atan2(self.z, math.sqrt(self.x * self.x + self.y * self.y)) - math.atan2(self.l4, self.l3)
        self.joint[5] = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z) - self.l1 - self.l5

        return self.joint

    def get_error(self, traget_point):
        tran_l = self.l2*math.sin(self.joint[1]) + self.l3*math.cos(self.joint[1] + self.joint[2] - math.pi/2)
        dx = traget_point[0] - self.carmera_midpoint[0]
        dy = traget_point[1] - self.carmera_midpoint[1]

        return tran_l, dx, dy
    
    def trans_joint_command(self, dx, dy):
        x_at_goal = False
        y_at_goal = False

        if abs(dy)<40:
            y_at_goal = True
        else:
            self.joint[0] = self.joint[0] + dy/abs(dy)*math.pi/180
            self.joint[5] = self.joint[5] - dy/abs(dy)*math.pi/180

        if abs(dx)<40:
            x_at_goal = True
        else:
            self.joint[1] = self.joint[1] - dx/abs(dx)*math.pi/180
            self.joint[2] = self.joint[2] + dx/abs(dx)*math.pi/180


        joint = self.joint

        return joint, x_at_goal, y_at_goal
    
    def gripper_grasp(self, train_l, image_range):
        z_at_goal = False
        if image_range[1]<900:
            self.joint[1] = self.joint[1] + math.pi/180  
            self.joint[2] = math.pi/2 - self.joint[1] + math.acos((train_l-self.l2*math.sin(self.joint[1]))/self.l3)
            self.joint[4] = math.pi-self.joint[1]-self.joint[2]
        
        else:
            z_at_goal = True
        
        joint = self.joint

        return joint , z_at_goal
    
    def grasp(self):
        self.gripper[0] = 0.0047
        self.gripper[1] = 0.0047

        gripper = self.gripper
        return gripper

        



