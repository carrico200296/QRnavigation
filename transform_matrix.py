import math
from  sympy import *
import numpy as np
# from patrol_smr import QR

def transform_matrix():
        
        Bx1=3.03 # x_info_qr_code_1_world frame
        By1=1.15 # y_info_qr_code 1_world frame

        Ax1= 2.1# x_position_qr_code 1_robot frame
        Ay1= -1.2# y_position_qr_code 1_robot frame

        #Distance between qr_1 and the origin of the world frame
        r1 = math.sqrt(math.pow(Bx1,2) + math.pow(By1,2))

        Bx2=2.7 # x_position_qr_code_2_world frame
        By2=3.2 # y_position_qr_code_2_world frame

        Ax2=4.4 # x_position_qr_code 1_robot frame
        Ay2=-1.0 # y_position_qr_code 1_robot frame

        #Distance between qr_2 and the origin of the world frame
        r2 = math.sqrt(math.pow(Bx2,2) + math.pow(By2,2))

        x, y  = symbols('x y')
        #Solution origin in world frame
        solutionA = solve([(x-Ax1)**(2) + (y-Ay1)**(2) - math.pow(r1,2),(x-Ax2)**(2) + (y-Ay2)**(2) - math.pow(r2,2)], [x,y])

        #Solution origin in robot frame
        solutionB = solve([(x-Bx1)**(2) + (y-By1)**(2) - math.pow(r1,2),(x-Bx2)**(2) + (y-By2)**(2) - math.pow(r2,2)], [x,y])

        # Once we know the origin coordinates of the QR_codes frame based on the robot frame{A}
        origin = 0
        for i in range(0,2):
                if origin == 0:
                        Axorigin = solutionA[i][0]
                        Ayorigin = solutionA[i][1]
                        # print(Axorigin,Ayorigin)

                #Rotation matrix - frame B from the frame A
                #Angle theta
                alpha = math.atan((Ay2 - Ayorigin)/(Ax2 - Axorigin))
                beta = math.atan(By2/Bx2)
                theta = alpha - beta


                A_T_B = np.array([[math.cos(theta), -(math.sin(theta)), Axorigin],
                        [math.sin(theta),  math.cos(theta), Ayorigin],
                        [    0      ,       0    ,     1  ]])

                #Check what point is the correct origin - we have to check if using
                #the origin coordinates we can get both points (PTO 1 and PTO 2
                #correctly) according to the robot frame {A}

                B_p1_no_transpose = np.array([Bx1, By1, 1])
                A_p1_no_transpose = np.array([Ax1, Ay1, 1])
                B_p1 = B_p1_no_transpose.transpose()
                A_p1 = A_p1_no_transpose.transpose()

                # print(A_p1[0])

                A_p1_test = A_T_B.dot(B_p1)  
                # print(A_p1_test)

                B_p2_no_transpose = np.array([Bx2, By2, 1])
                A_p2_no_transpose = np.array([Ax2, Ay2, 1])
                B_p2 = B_p2_no_transpose.transpose()
                A_p2 = A_p2_no_transpose.transpose()

                A_p2_test = A_T_B.dot(B_p2)


                if A_p1_test[0]==A_p1[0] and A_p1_test[1]==A_p1[1] and A_p2_test[0]==A_p2[0] and A_p2_test[1]==A_p2[1]:
                        origin=1


                # Bx3 = 4
                # By3 = 6

                # B_p3 = np.array([Bx3, By3, 1])
                # A_p3 = A_T_B.dot(B_p3)

        # print(A_T_B)
        return A_T_B