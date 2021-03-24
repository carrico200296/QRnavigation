import math
from  sympy import *
import numpy as np
from patrol_smr import QR

def next_qr_position(tf_matrix, last_qr):

    x_next_qr_world_frame =  last_qr.nx # x_qr_next (from the qr_previous)
    y_next_qr_world_frame =  last_qr.ny # y_qr_next (from the qr_previous)

    next_qr_world_frame = np.array([x_next_qr_world_frame, y_next_qr_world_frame, 1])
    next_qr_robot_frame = tf_matrix.dot(next_qr_world_frame)

    return next_qr_robot_frame