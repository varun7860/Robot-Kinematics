#! /usr/bin/env python

import rospy
import math
import numpy as np

from numpy import *

class Forward_Kinematics_Solver(object):

     def __init__(self,angle_1,angle_2,d_3):

          ### Link Lengths in cm ####
          self.l1 = 0.5
          self.l2 = 0.4 #a2
          self.l3 = -0.2
          self.a1 = 0.4    #input

          ### Angles ###
          self.theta_1 = angle_1
          self.theta_2 = angle_2
          self.d_3 = d_3

          ### Rotation around X-axis ###
          self.alpha_1 = 0.0
          self.alpha_2 = 0.0
          self.alpha_3 = 0.0

          ### Distance between x-axis of two frames ###
          '''This Parameter is denoted by r'''

          self.r1 = self.a1
          self.r2 = self.l2
          self.r3 = 0.0

          ### Distance between z-axis of two frames ###
          '''This parameter is denoted by d'''

          self.d1 = self.l1
          self.d2 = 0.0
          self.d3 = self.l3 + self.d_3

          ##### Convert the angles to Radians #####
          self.theta_1 = (self.theta_1/180.0)*np.pi
          self.theta_2 = (self.theta_2/180.0)*np.pi
          self.theta_3 = 0

           self.alpha_1 = (self.alpha_1/180.0)*np.pi
           self.alpha_2 = (self.alpha_2/180.0)*np.pi
           self.alpha_3 = (self.alpha_3/180.0)*np.pi

          ### DH Parameter Table ###
          self.D = [[self.theta_1,self.alpha_1,self.r1,self.d1],
                    [self.theta_2,self.alpha_2,self.r2,self.d2],
                    [self.theta_3,self.alpha_3,self.r3,self.d3]]

          # print(self.D)


     def compute_transformation_matrix(self,frame,i):

          D = self.D

          if frame == "f0_1":

               T = [[np.cos(D[i][0]),-np.sin(D[i][0])*np.cos(D[i][1]),np.sin(D[i][0])*np.sin(D[i][1]),D[i][2]*np.cos(D[i][0])],
                    [np.sin(D[i][0]),np.cos(D[i][0])*np.cos(D[i][1]),-np.cos(D[i][0])*np.sin(D[i][1]),D[i][2]*np.sin(D[i][0])],
                    [0,np.sin(D[i][1]),np.cos(D[i][1]),D[i][3]],
                    [0,0,0,1]]


          elif frame == "f1_2":

               T = [[np.cos(D[i][0]),-np.sin(D[i][0])*np.cos(D[i][1]),np.sin(D[i][0])*np.sin(D[i][1]),D[i][2]*np.cos(D[i][0])],
                    [np.sin(D[i][0]),np.cos(D[i][0])*np.cos(D[i][1]),-np.cos(D[i][0])*np.sin(D[i][1]),D[i][2]*np.sin(D[i][0])],
                    [0,np.sin(D[i][1]),np.cos(D[i][1]),D[i][3]],
                    [0,0,0,1]]

          elif frame == "f2_3":

               T = [[np.cos(D[i][0]),-np.sin(D[i][0])*np.cos(D[i][1]),np.sin(D[i][0])*np.sin(D[i][1]),D[i][2]*np.cos(D[i][0])],
                    [np.sin(D[i][0]),np.cos(D[i][0])*np.cos(D[i][1]),-np.cos(D[i][0])*np.sin(D[i][1]),D[i][2]*np.sin(D[i][0])],
                    [0,np.sin(D[i][1]),np.cos(D[i][1]),D[i][3]],
                    [0,0,0,1]]

          return T

     def compute_homogeneous_transformation_matrix(self,T0_1,T1_2,T2_3):

          self.H0_1 = dot(T0_1,T1_2)
          self.H0_3 = dot(self.H0_1,T2_3)

          #print("Homogenous Transformation Matrix:")
          #print(self.H0_3)
          #print("\n")
          #print("X,Y,Z Cordinates of End Effector: ", (self.H0_3[0][3], self.H0_3[1][3], self.H0_3[2][3]))

          return self.H0_3


class Inverse_Kinematics_Solver(object):
     def __init__(self,x,y,z):
          #### End Effector Cordinates ####
          self.x = x
          self.y = y
          self.z = z

          ####Link Lengths####
          self.l1 = 0.5 #d1
          self.l2 = 0.4 #a2
          self.l3 = -0.2 #a3

          #update
          self.a1 =  0.4  #input

          #### Joint Angles ####
          self.j1 = None
          self.j2 = None

          self.j3 = None

          #### Geometrical variables ####
          self.phi_1 = None
          self.phi_2 = None
          self.phi_3 = None
          self.r1 = None
          self.r2 = None
          self.r3 = None     

          #update
          self.D = None
          self.d3 = None 

     def solve_inverse_kinematics(self):
      
          self.D = (((self.x)**2) + ((self.y)**2) - ((self.a1)**2) - ((self.l2)**2))/ (2*(self.a1)*(self.l2))
          self.phi_2 = math.atan2(math.sqrt(abs(1-((self.D)**2))),self.D)

          self.phi_1 = math.atan2(self.y,self.x) - math.atan2((self.l2*math.sin(self.phi_2)),(self.a1 + (self.l2*math.cos(self.phi_2))))

          self.d3 = -(self.l1 - self.z + self.l3)  
          

     def get_joint_angles(self):

          self.j1 = self.phi_1
          self.j2 = self.phi_2


          self.j1 = round((self.j1*180)/3.142)
          self.j2 = round((self.j2*180)/3.142)
          self.q3 = self.d3

          return self.j1, self.j2, self.q3
