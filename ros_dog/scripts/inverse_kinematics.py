import math


def inverse_kinematics(x, y, L1, L2):
    # 计算到目标点的距离
    distance = math.sqrt(x**2 + y**2)

    # 计算关节2的角度
    cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = math.sqrt(1 - cos_theta2**2)
    theta2 = math.atan2(sin_theta2, cos_theta2)

    # 计算关节1的角度
    alpha = math.atan2(y, x)
    beta = math.acos((distance**2 + L1**2 - L2**2) / (2 * distance * L1))
    theta1 = alpha - beta

    return theta1, theta2


# 机械臂参数
L1 = 3.0  # 关节1到关节2的长度
L2 = 2.0  # 关节2到末端的长度

# 目标末端位置
target_x = 2.0
target_y = 2.0

# 求解逆运动学
theta1, theta2 = inverse_kinematics(target_x, target_y, L1, L2)

print("Joint angles (in radians):")
print("Theta1:", theta1)
print("Theta2:", theta2)
