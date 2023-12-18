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
        self.deg_to_rad = math.pi / 180  # Precompute degrees to radians conversion

    def get_error(self, target_point):
        tran_l = self.l2 * math.sin(self.joint[1]) + self.l3 * math.cos(self.joint[1] + self.joint[2] - math.pi/2)
        dx = target_point[0] - self.carmera_midpoint[0]
        dy = target_point[1] - self.carmera_midpoint[1]

        return tran_l, dx, dy
    
    def trans_joint_command(self, dx, dy):
        x_at_goal = False
        y_at_goal = False

        if abs(dy) < 40:
            y_at_goal = True
        else:
            angle_change = dy / abs(dy) * self.deg_to_rad
            self.joint[0] += angle_change
            self.joint[5] -= angle_change

        if abs(dx) < 40:
            x_at_goal = True
        else:
            angle_change = dx / abs(dx) * self.deg_to_rad
            self.joint[1] -= angle_change
            self.joint[2] += angle_change

        joint = self.joint
        return joint, x_at_goal, y_at_goal
    
    def gripper_grasp(self, train_l, image_range):
        z_at_goal = False
        if image_range[1] < 900:
            self.joint[1] += self.deg_to_rad
            self.joint[2] = math.pi / 2 - self.joint[1] + math.acos((train_l - self.l2 * math.sin(self.joint[1])) / self.l3)
            self.joint[4] = math.pi - self.joint[1] - self.joint[2]
        else:
            z_at_goal = True
        
        joint = self.joint
        return joint, z_at_goal
    
    def grasp(self):
        self.gripper[0] = 0.005
        self.gripper[1] = 0.005
        gripper = self.gripper
        return gripper
