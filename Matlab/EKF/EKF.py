#EKF bois

import numpy;
import math;

class EKF(object):
    def __init__(self):
        self.A = numpy.identity(5)
        self.R = numpy.zeros((5,5))
        self.R[0][0] = 400 # GPS covariances
        self.R[1][1] = 400
        self.R[2][2] = 0.01 # IMU covariance
        self.R[3][3] = 0.04 # Encoder covariances
        self.R[4][4] = 0.04
        self.Q = numpy.identity(5)
        self.Q = 0.1*self.Q
        self.H = numpy.identity(5)
        self.xest = numpy.zeros(5).transpose()
        self.x = numpy.zeros(5).transpose()
        self.Pest = numpy.identity(5)
        self.P = numpy.identity(5)
        self.P = 100*self.P
        self.l = 57.15



    def step(self, *args):
        if(len(args) == 6):
            timestep, gpsX, gpsY, imuHeading, thetaR, thetaL = args
            #Prediction Equations
            self.A[0][3] = -0.5*math.sin(self.x[2])
            self.A[0][4] = -0.5*math.sin(self.x[2])
            self.A[1][3] = 0.5*math.cos(self.x[2])
            self.A[1][4] = 0.5*math.cos(self.x[2])
            self.A[2][3] = 1/self.l
            self.A[2][4] = -1/self.l
            self.xest = self.A.dot(self.x)
            #print("Previous Vr " + str(float(self.x[3])))
            #print("Previous Vl " + str(float(self.x[4])))
            #print("Previous IMU Heading " + str(float(self.x[2])))
            #print("Estimated IMU Heading" + str(float(self.xest[2])))
            self.Pest = self.A.dot(self.P.dot(self.A.transpose())) + self.Q

            #Measurement Update
            z = numpy.array([gpsX, gpsY, imuHeading, thetaR, thetaL]).transpose()
            self.H = numpy.identity(5)
            self.H[0][0] = 1
            self.H[1][1] = 1
            self.H[2][3] = 1/(timestep*self.l)
            self.H[2][4] = -1/(timestep*self.l)
            self.H[3][3] = 1
            self.H[4][4] = 1

            #self.H[3][0] = 1
            #self.H[4][1] = 1
            #Measurement Correction
            I = numpy.identity(5)
            try:
                inv = numpy.linalg.inv(self.H.dot(self.Pest.dot(self.H.transpose()))+self.R)
            except:
                inv = numpy.identity(5)
            K = self.Pest.dot(self.H.transpose()).dot(inv)
            self.x = self.xest + K.dot(z - self.H.dot(self.xest))
            self.P = (I - K.dot(self.H)).dot(self.Pest)
        elif (len(args) == 4):
            timestep, imuHeading, thetaR, thetaL = args
            # Prediction Equations
            self.A[0][3] = -0.5 * math.sin(self.x[2])
            self.A[0][4] = -0.5 * math.sin(self.x[2])
            self.A[1][3] = 0.5 * math.cos(self.x[2])
            self.A[1][4] = 0.5 * math.cos(self.x[2])
            self.A[2][3] = 1 / self.l
            self.A[2][4] = -1 / self.l
            self.xest = self.A.dot(self.x)
            # print("Previous Vr " + str(float(self.x[3])))
            # print("Previous Vl " + str(float(self.x[4])))
            # print("Previous IMU Heading " + str(float(self.x[2])))
            # print("Estimated IMU Heading" + str(float(self.xest[2])))
            self.Pest = self.A.dot(self.P.dot(self.A.transpose())) + self.Q

            # Measurement Update
            z = numpy.array([0, 0, imuHeading, thetaR, thetaL]).transpose()
            self.H = numpy.identity(5)
            self.H[0][0] = 10000
            self.H[1][1] = 10000
            self.H[2][3] = 1 / (timestep * self.l)
            self.H[2][4] = -1 / (timestep * self.l)

            # self.H[3][0] = 1
            # self.H[4][1] = 1
            # Measurement Correction
            I = numpy.identity(5)
            try:
                inv = numpy.linalg.inv(self.H.dot(self.Pest.dot(self.H.transpose())) + self.R)
            except:
                inv = 0.1*numpy.identity(5)
            K = self.Pest.dot(self.H.transpose()).dot(inv)
            self.x = self.xest + K.dot(z - self.H.dot(self.xest))
            self.P = (I - K.dot(self.H)).dot(self.Pest)

