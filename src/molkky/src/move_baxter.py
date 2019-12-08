# import numpy as np
#
# def baxter_make_move(move):
#     print("MOVE")
#     # TODO QUENTIN/BIANCA
#     return None
#
# def baxter_throw():
#     print("THROW")
#     # TODO WILLIAM
#     return True
#
# def shot(row, col):
#     g = 9.8
#     v = 10 # change according to pneumatic release speed
#     h = 10 # change according to height of gripper (should we make this variable?)
#     pitch = 0
#     yaw = 0
#
#     x = row # change according to how we partition grid
#     y = col # change according to how we partition grid
#
#     pitch = np.asin(x * g / (v * v)) / 2
#     yaw = atan (x / y)
#
#     return (pitch, yaw)
