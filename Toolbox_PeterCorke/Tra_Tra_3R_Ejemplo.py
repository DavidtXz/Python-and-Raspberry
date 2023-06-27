from roboticstoolbox import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import math
import numpy as np
from spatialmath.base import *
from spatialmath import *
import matplotlib as plt
import matplotlib.pyplot as plt

class Robot3R(DHRobot):
    def __init__(self):
        phome = [27.4, 0, 6.5]
        pcam = [18.253, 0, 26.81626]
        pfig = [23.651, 2.069, 1.74]
        l1 = 6.5
        l2 = 8.4
        l3 = 19
        R = []
        R.append(RevoluteDH(d=l1, a=0, alpha=math.pi / 2, offset=0))
        R.append(RevoluteDH(d=0, a=l2, alpha=0, offset=0))
        R.append(RevoluteDH(d=0, a=l3, alpha=0, offset=0))
        super().__init__(R, name="3R", keywords=("angular",))
        self.qt1 = 0
        self.qt2 = 0
        self.qt3 = 0
        
    def IK(self, l1, l2, l3, p):  # Cinematica Inversa
        b = math.sqrt(p[0] ** 2 + p[1] ** 2)
        c = p[2] - l1
        e = math.sqrt(b ** 2 + c ** 2)
        theta1 = math.atan2(p[1], p[0])
        cost3 = (e ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3)
        sent3 = -math.sqrt(1 - (cost3) ** 2)
        theta3 = math.atan2(sent3, cost3)
        alpha = math.atan2(c, b)
        phi = math.atan2((l3 * sent3), (l2 + l3 * cost3))
        theta2 = alpha - phi
        theta1 = round(theta1, 5)
        theta2 = round(theta2, 5)
        theta3 = round(theta3, 5)
#         if (theta2 >= -46*math.pi/180 and theta2 <= math.pi):
#             theta2 = round(theta2 + math.pi / 4, 5)
#         if (theta3 >= -91*math.pi/180 and theta3 <= math.pi):
#             theta3 = round(theta3 + math.pi / 2, 5)
        #print('theta1: ' + str(round(math.degrees(theta1),5)) + ' ---- ' + str(theta1))
        #print('theta2: ' + str(round(math.degrees(theta2),5)) + ' ---- ' + str(theta2))
        #print('theta3: ' + str(round(math.degrees(theta3),5)) + ' ---- ' + str(theta3))
        return theta1, theta2, theta3        
        
    def tra_tra(self, puntoI, puntoF, numDatos):
        #Tra_Tra(phome, pcam, 30)
#         [t1_1, t2_1, t3_1] = IK(l1, l2, l3, puntoI[0], puntoI[1], puntoI[2]);
#         [t1_2, t2_2, t3_2] = IK(l1, l2, l3, puntoF[0], puntoF[1], puntoI[2]);
        t = np.linspace(0, 1, numDatos)
        [theta1_P1, theta2_P1, theta3_P1] = self.IK(l1, l2, l3, puntoI)
        [theta1_P2, theta2_P2, theta3_P2] = self.IK(l1, l2, l3, puntoF)
        if theta1_P1 == theta1_P2:
            theta1_P2 += 0.0001
        if theta2_P1 == theta2_P2:
            theta2_P2 += 0.0001
        if theta3_P1 == theta3_P2:
            theta3_P2 += 0.0001
        self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
        self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
        self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
        for i in range(numDatos):
            robot.plot([self.qt1.q[i], self.qt2.q[i], self.qt3.q[i]], limits=[-25, 25, -25, 25, 0, 40])
            if i < numDatos-1:
                plt.clf()
#         self.qt1.plot()
#         self.qt2.plot()
#         self.qt3.plot()
    def tra_tra2(self, puntoI, puntoF, numDatos):
        t = np.linspace(0, 1, numDatos)
        [theta1_P1, theta2_P1, theta3_P1] = self.IK(l1, l2, l3, puntoI)
        [theta1_P2, theta2_P2, theta3_P2] = self.IK(l1, l2, l3, puntoF)
        if theta1_P1 == theta1_P2:
            theta1_P2 += 0.0001
        if theta2_P1 == theta2_P2:
            theta2_P2 += 0.0001
        if theta3_P1 == theta3_P2:
            theta3_P2 += 0.0001
        self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
        self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
        self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
        for i in range(numDatos):
            #print("ServoA :" + str(np.round(self.qt1.q[i],5)) + "    " + "ServoB :" + str(np.round(self.qt2.q[i],5)) + "    " + "ServoC :" + str(np.round(self.qt3.q[i],5)))
            print("ServoA :" + str(np.round(np.rad2deg(self.qt1.q[i]),2)) + "    " + "ServoB :" + str(np.round(np.rad2deg(self.qt2.q[i]),2)) + "    " + "ServoC :" + str(np.round(np.rad2deg(self.qt3.q[i]),2)))
            robot.plot([self.qt1.q[i], self.qt2.q[i], self.qt3.q[i]], limits=[-25, 25, -25, 25, 0, 40])
            if i < numDatos-1:
                plt.clf()
    def tra_tra3(self, puntoI, puntoF, numDatos):
        a = 1
        [theta1_P1, theta2_P1, theta3_P1] = self.IK(l1, l2, l3, puntoI)
        [theta1_P2, theta2_P2, theta3_P2] = self.IK(l1, l2, l3, puntoF)
        if theta1_P1 == theta1_P2:
            theta1_P2 += 0.0001
        if theta2_P1 == theta2_P2:
            theta2_P2 += 0.0001
        if theta3_P1 == theta3_P2:
            theta3_P2 += 0.0001
            
        if a == 0: #Sin sensores para velocidad
            t = np.linspace(0, 1, numDatos)
            self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
            self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
            self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
            for i in range(numDatos):
                #print("ServoA :" + str(np.round(self.qt1.q[i],5)) + "    " + "ServoB :" + str(np.round(self.qt2.q[i],5)) + "    " + "ServoC :" + str(np.round(self.qt3.q[i],5)))
                print("ServoA :" + str(np.round(np.rad2deg(self.qt1.q[i]),2)) + "    " + "ServoB :" + str(np.round(np.rad2deg(self.qt2.q[i]),2)) + "    " + "ServoC :" + str(np.round(np.rad2deg(self.qt3.q[i]),2)))
                robot.plot([self.qt1.q[i], self.qt2.q[i], self.qt3.q[i]], limits=[-25, 25, -25, 25, 0, 40])
                if i < numDatos-1:
                    plt.clf()
        elif a == 1: #Con sensores para velocidad
            sensor1 = 1
            sensor2 = 1
            sensor3 = 1
            [theta1_P1, theta2_P1, theta3_P1] = self.IK(l1, l2, l3, puntoI)
            [theta1_P2, theta2_P2, theta3_P2] = self.IK(l1, l2, l3, puntoF)
            if theta1_P1 == theta1_P2:
                theta1_P2 += 0.0001
            if theta2_P1 == theta2_P2:
                theta2_P2 += 0.0001
            if theta3_P1 == theta3_P2:
                theta3_P2 += 0.0001
            t = np.linspace(0, 1, numDatos)
            self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
            self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
            self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
            
            for i in range(numDatos):
                if sensor1 == 0:
                    while sensor1 == 0:
                        print("Robot detenido")
                        if sensor1 == 1:
                            print("Trayectoria normal")
                            break
                if i == 7:
                    sensor2 = 0
                if sensor2 == 0:
                    print("Velocidad reducida")
                    t = np.linspace(0, 1, int(numDatos * 2.5))
                    theta1_P1 = self.qt1.q[i-1]
                    theta2_P1 = self.qt2.q[i-1]
                    theta3_P1 = self.qt3.q[i-1]
                    self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
                    self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
                    self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
                    for j in range(int(numDatos * 2.5)):
                        
                        if j == 20:
                            sensor2 = 1
                        if sensor2 == 1:
                            print("Trayectoria normal")
                            t = np.linspace(0, 1, int(numDatos / 3))
                            theta1_P1 = self.qt1.q[j-1]
                            theta2_P1 = self.qt2.q[j-1]
                            theta3_P1 = self.qt3.q[j-1]
                            self.qt1 = trapezoidal(theta1_P1,theta1_P2,t)
                            self.qt2 = trapezoidal(theta2_P1,theta2_P2,t)
                            self.qt3 = trapezoidal(theta3_P1,theta3_P2,t)
                            for k in range(int(numDatos / 3)):
                                #print("ServoA :" + str(np.round(self.qt1.q[k],5)) + "    " + "ServoB :" + str(np.round(self.qt2.q[k],5)) + "    " + "ServoC :" + str(np.round(self.qt3.q[k],5)))
                                print("ServoA :" + str(np.round(np.rad2deg(self.qt1.q[k]),2)) + "    " + "ServoB :" + str(np.round(np.rad2deg(self.qt2.q[k]),2)) + "    " + "ServoC :" + str(np.round(np.rad2deg(self.qt3.q[k]),2)))
                                robot.plot([self.qt1.q[k], self.qt2.q[k], self.qt3.q[k]], limits=[-25, 25, -25, 25, 0, 40])
                                if i < (int(numDatos / 3))-1:
                                    plt.clf()
                            break         
                        #print("ServoA :" + str(np.round(self.qt1.q[j],5)) + "    " + "ServoB :" + str(np.round(self.qt2.q[j],5)) + "    " + "ServoC :" + str(np.round(self.qt3.q[j],5)))
                        print("ServoA :" + str(np.round(np.rad2deg(self.qt1.q[j]),2)) + "    " + "ServoB :" + str(np.round(np.rad2deg(self.qt2.q[j]),2)) + "    " + "ServoC :" + str(np.round(np.rad2deg(self.qt3.q[j]),2)))
                        robot.plot([self.qt1.q[j], self.qt2.q[j], self.qt3.q[j]], limits=[-25, 25, -25, 25, 0, 40])
                        if i < (int(numDatos * 2.5))-1:
                            plt.clf()
                    break
                #print("ServoA :" + str(np.round(self.qt1.q[i],5)) + "    " + "ServoB :" + str(np.round(self.qt2.q[i],5)) + "    " + "ServoC :" + str(np.round(self.qt3.q[i],5)))
                print("ServoA :" + str(np.round(np.rad2deg(self.qt1.q[i]),2)) + "    " + "ServoB :" + str(np.round(np.rad2deg(self.qt2.q[i]),2)) + "    " + "ServoC :" + str(np.round(np.rad2deg(self.qt3.q[i]),2)))
                robot.plot([self.qt1.q[i], self.qt2.q[i], self.qt3.q[i]], limits=[-25, 25, -25, 25, 0, 40])
                if i < numDatos-1:
                    plt.clf()
        
if __name__ == "__main__":
    robot = Robot3R()
    print(robot)
    #t = np.linspace(0, 1, numDatos)
    phome = [27.4, 0, 6.5]
    pcam = [18.253, 0, 26.81626]
    pfig = [23.651, 2.069, 1.74]
    l1 = 6.5
    l2 = 8.4
    l3 = 19
    #robot.tra_tra(phome, pcam, 40)
    #robot.tra_tra2(pcam, pfig, 15)
    #robot.tra_tra3(pcam, pfig, 20)
    robot.tra_tra3(pcam,pfig,30)
    robot.qt1.plot()
    

    

    
