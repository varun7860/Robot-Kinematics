#! /usr/bin/env python

import numpy as np
from kinematics_solver import Forward_Kinematics_Solver

class JacobianGenerator(object):
    
    def __init__(self,t1,t2,t3):

        #Declare fk solver
        self.fk_solver = Forward_Kinematics_Solver(t1,t2,t3)

        #Jacobian Matrix
        self.J = np.zeros((6,3),np.float64)

        #Initialize Rotation Matrix
        self.R = np.zeros((3,3),np.float64)

        #Initialize Displacement Vector
        self.D = np.matrix([0,0,0])

    def get_rotation_matrix(self,matrix):
        if matrix == "R00":
            self.R = [[1,0,0],
                      [0,1,0],
                      [0,0,1]]
            
            self.R = np.matrix(self.R)

        elif matrix == "R01":
            HR = self.fk_solver.compute_transformation_matrix("f0_1",0)
            
            self.R = [[HR[0][0],HR[0][1],HR[0][2]],
                      [HR[1][0],HR[1][1],HR[1][2]],
                      [HR[2][0],HR[2][1],HR[2][2]]]

        elif matrix == "R02":
            HR01 = self.fk_solver.compute_transformation_matrix("f0_1",0)
            HR12 = self.fk_solver.compute_transformation_matrix("f1_2",1)
            HR = np.dot(HR01,HR12)

            self.R = [[HR[0][0],HR[0][1],HR[0][2]],
                      [HR[1][0],HR[1][1],HR[1][2]],
                      [HR[2][0],HR[2][1],HR[2][2]]]
        
        else:
            print("Error! Wrong Rotation Matrix")

        return self.R

    def get_displacement_vector(self,vector):

        if vector == "D00":
            pass

        elif vector == "D01":
            HR = self.fk_solver.compute_transformation_matrix("f0_1",0)
            self.D = np.matrix([HR[0][3],HR[1][3],HR[2][3]])

        elif vector == "D02":
            HR01 = self.fk_solver.compute_transformation_matrix("f0_1",0)
            HR12 = self.fk_solver.compute_transformation_matrix("f1_2",1)
            HR = np.dot(HR01,HR12)
            self.D = np.matrix([HR[0][3],HR[1][3],HR[2][3]])

        elif vector == "D03":
            HR01 = self.fk_solver.compute_transformation_matrix("f1_2",1)
            HR12 = self.fk_solver.compute_transformation_matrix("f2_3",2)
            HR = np.dot(HR01,HR12)
            self.D = np.matrix([HR[0][3],HR[1][3],HR[2][3]])

        else:
            print("Error!. Wrong Displacement Vector")

        return self.D

    def calculate_prismatic_jacobian_linear(self,R):
        p_jacobian = np.dot(R,np.transpose(np.matrix([0,0,1])))
        return p_jacobian

    def calculate_prismatic_jacobian_rotational(self):
        p_jacobian = np.transpose(np.matrix([0,0,0]))
        return p_jacobian

    def calculate_revolute_jacobian_linear(self,R,Dn,Di):
        D = Dn-Di
        R = np.dot(R,np.transpose(np.matrix([0,0,1])))
        R = np.transpose(R)
        r_jacobian = np.cross(R,D)
        r_jacobian = np.transpose(r_jacobian)
        return r_jacobian

    def calculate_revolute_jacobian_rotational(self,R):
        r_jacobian = np.dot(R,np.transpose(np.matrix([0,0,1])))
        return r_jacobian

    def update_jacobian_matrix(self,val,column_num,loc):
       
        val_1 = val[0]
        val_2 = val[1]
        val_3 = val[2]

        if loc == "upper":
            self.J[0,column_num] = val_1
            self.J[1,column_num] = val_2
            self.J[2,column_num] = val_3

        elif loc == "lower":
            self.J[3,column_num] = val_1
            self.J[4,column_num] = val_2
            self.J[5,column_num] = val_3

        else:
            print("invalid location of the matrix")

    def display_jacobian_matrix(self):
        print("Jacobian Matrix",self.J)

    def get_jacobian_matrix(self):
        return self.J

        
class VelocityFkSolver(object):
    def __init__(self,t1_dot,t2_dot,t3_dot):

        #Declare the joint velocities
        self.q1_dot = t1_dot
        self.q2_dot = t2_dot
        self.d3_dot = t3_dot
        print("Joint Velocities:",self.q1_dot,self.q2_dot,self.d3_dot)

        #Cartesian Velocities
        self.x_dot = None
        self.y_dot = None
        self.z_dot = None
        self.wx_dot = None
        self.wy_dot = None
        self.wz_dot = None
        self.V = None
    
    def get_cartesian_velocities(self,J):

        self.V = np.dot(J,np.transpose([self.q1_dot,self.q2_dot,self.d3_dot]))

        self.x_dot = self.V[0]
        self.y_dot = self.V[1]
        self.z_dot = self.V[2]
        self.wx_dot = self.V[3]
        self.wy_dot = self.V[4]
        self.wz_dot = self.V[5]

        return self.x_dot,self.y_dot,self.z_dot,self.wx_dot,self.wy_dot,self.wz_dot


class VelocityIkSolver(object):

    def __init__(self,x,y,z,wx,wy,wz):

        #Cartesian Velocities
        self.x_dot = x
        self.y_dot = y
        self.z_dot = z
        self.wx_dot = wx
        self.wy_dot = wy
        self.wz_dot = wz

        #Cartesian Velocity 6x1 Matrix
        self.CV = np.transpose([[self.x_dot,self.y_dot,self.z_dot,self.wx_dot,self.wy_dot,self.wz_dot]])

        #Joint Velocities
        self.q1_dot = None
        self.q2_dot = None
        self.q3_dot = None


    def get_joint_velocities(self,J):

        self.JV = np.dot(np.linalg.pinv(J),self.CV)
        
        self.q1_dot = self.JV[0]
        self.q2_dot = self.JV[1]
        self.q3_dot = self.JV[2]

        return self.q1_dot, self.q2_dot, self.q3_dot


def main():
    '''
    #Create the solver object
    generator = JacobianGenerator(0.0,0.0,0.0)
    vk_fk_solver = VelocityFkSolver(1.5,3.5,1.2)
    vk_ik_solver = VelocityIkSolver(0,0.6,1.2,0.0,0.0,5.0)


    #Find the R and D parameters
    R00 = generator.get_rotation_matrix("R00")
    R01 = generator.get_rotation_matrix("R01")
    R02 = generator.get_rotation_matrix("R02")
    D00 = generator.get_displacement_vector("D00")
    D01 = generator.get_displacement_vector("D01")
    D02 = generator.get_displacement_vector("D02")
    D03 = generator.get_displacement_vector("D03")

    #Find the jacobian parameters
    rj0_upper = generator.calculate_revolute_jacobian_linear(R00,D03,D00)
    rj0_lower = generator.calculate_revolute_jacobian_rotational(R00)
    rj1_upper = generator.calculate_revolute_jacobian_linear(R01,D03,D01)
    rj1_lower = generator.calculate_revolute_jacobian_rotational(R01)
    pj2_upper = generator.calculate_prismatic_jacobian_linear(R02)
    pj2_lower = generator.calculate_prismatic_jacobian_rotational()

    #Update the Jacobian Matrix
    generator.update_jacobian_matrix(rj0_upper,0,"upper")
    generator.update_jacobian_matrix(rj0_lower,0,"lower")
    generator.update_jacobian_matrix(rj1_upper,1,"upper")
    generator.update_jacobian_matrix(rj1_lower,1,"lower")
    generator.update_jacobian_matrix(pj2_upper,2,"upper")
    generator.update_jacobian_matrix(pj2_lower,2,"lower")

    #Display Jacobian Matrix
    generator.display_jacobian_matrix()
    JACOBIAN_MATRIX = generator.get_jacobian_matrix()

    #Get the Cartesian Velocities from joint velocities
    a,b,c,d,e,f = vk_fk_solver.get_cartesian_velocities(JACOBIAN_MATRIX)
    print("Cartesian Velocities:",a,b,c,d,e,f)

    #Get Joint Velocities from cartesian Velocities
    q1_dot,q2_dot,d3_dot = vk_ik_solver.get_joint_velocities(JACOBIAN_MATRIX)
    print("Joint Velocities:",q1_dot,q2_dot,d3_dot)
    '''

if __name__ == '__main__':
    main()

       
        
