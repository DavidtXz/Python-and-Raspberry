from roboticstoolbox import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import math
import numpy as np
from spatialmath.base import *
from spatialmath import *
import matplotlib as plt
import matplotlib.pyplot as plt

class Robot2R(DHRobot):
    def __init__(self):
        phome = [27.4, 0, 6.5]
        pcam = [18.253, 0, 26.81626]
        pfig = [23.651, 2.069, 1.74]
        l1 = 5.0/1;
        l2 = 5.0/1;
        R = []
        R.append(RevoluteDH(d=0, a=l1, alpha=0, offset=0))
        R.append(RevoluteDH(d=0, a=l2, alpha=0, offset=0))
        super().__init__(R, name="2R", keywords=("planar",))
        
    def IK(self,l1,l2,px,py):
        b = math.sqrt(px**2 + py**2)
        cos_theta2 = (b**2-l2**2-l1**2)/(2*l2*l1);
        sen_theta2 = math.sqrt(1-(cos_theta2)**2); #En esta linea se modifica con un '-' para la solucion codo arriba,
                                             #o codo abajo, negativo es codo arriba y positivo es codo abajo.
        theta2 = math.atan2(sen_theta2,cos_theta2);
        alpha = math.atan2(py,px);
        phi = math.atan2(l2*sen_theta2,(l1+l2*cos_theta2));
        theta1 = alpha-phi;
        if theta1 <= -math.pi:
            theta1 = (2*math.pi) + theta1
        #print("theta1: " + str(theta1) +"\ntheta2: " + str(theta2))
        return theta1, theta2
        
        
    def Tra_Tra(puntoI, puntoF, NumDatos):
        #Tra_Tra(phome, pcam, 30)
        [t1_1, t2_1, t3_1] = IK(l1, l2, l3, puntoI[0], puntoI[1], puntoI[2]);
        [t1_2, t2_2, t3_2] = IK(l1, l2, l3, puntoF[0], puntoF[1], puntoI[2]);
        
        
        
if __name__ == "__main__":
    robot = Robot2R()
    print(robot)
    numDatos = 40
    t = np.linspace(0, 1, numDatos)
    t1 = [0, 0]
    t2 = [0, 0]
    l1 = 5
    l2 = 5
    p = [[10,0],[-5,5],[-10,0]]
    p1 = [10, 0]
    p2 = [-5, 5]
    p3 = [-10, 0]
    #robot.plot([0,0,0,0,0])
    #robot.plot([0,0,0,0,0], limits=[-25, 25, -25, 25, 0, 40])
    #robot.teach([0,0])    #Instruccion para plotear robot con los sliders
    for i in range(2):
        print('***********PUNTO ' + str(i + 1) + ' ************')
        [t1[i], t2[i]] = robot.IK(l1, l2, p1[0], p1[1])
    
    
    [theta1_P1, theta2_P1] = robot.IK(5,5,10,0)
    [theta1_P2, theta2_P2] = robot.IK(5,5,-5,5)
    
    qt1 = trapezoidal(0,0.1,t)
    qt2 = trapezoidal(theta2_P1,theta2_P2,t,2)
    for i in range(numDatos):
        robot.plot([qt1.q[i], qt2.q[i]], limits=[-11, 11, -11, 11, -1, 40])
        if i < numDatos-1:
            plt.clf()
    
    qt1.plot()
    qt2.plot()
    plt.show()
    
