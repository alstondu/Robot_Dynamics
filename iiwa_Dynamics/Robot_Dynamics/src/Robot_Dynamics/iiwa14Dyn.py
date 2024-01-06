#!/usr/bin/env python3

import numpy as np
from Robot_Dynamics.iiwa14DynBase import Iiwa14DynamicBase


class Iiwa14DynamicRef(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicRef, self).__init__(tf_suffix='ref')

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # 1. T_rot_z * T_trans * T_rot_x * T_rot_y.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Given the joint values of the robot, compute the Jacobian matrix at the centre of mass of the link.

        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame to compute the Jacobian.
            Defaults to 7.

        Returns:
            jacobian (numpy.ndarray): The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the
            centre of mass of a link.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        # 1. Compute the forward kinematics (T matrix) for all the joints.
        # 2. Compute forward kinematics at centre of mass (T_cm) for all the joints.
        # 3. From the computed forward kinematic and forward kinematic at CoM matrices,
        # extract z, z_cm (z axis of the rotation part of T, T_cm) and o, o_cm (translation part of T, T_cm) for all links.
        # 4. Based on the computed o, o_cm, z, z_cm, fill J_p and J_o matrices up until joint 'up_to_joint'.
        # 5. Fill the remaining part with zeroes and return the Jacobian at CoM.

        # Initialize the Jacobian matrix
        jacobian = np.zeros((6, 7))
        # Forward kinematics at centre of mass
        T_cm = self.forward_kinematics_centre_of_mass(joint_readings, up_to_joint)
        # Extract p_li
        p_li = T_cm[0:3,3]
        for i in range(up_to_joint):
            # Forward kinematics
            T = self.forward_kinematics(joint_readings, i)
            # Extract z_j-1
            z_pre = T[:3, 2]
            # Extract p_j-1
            p_pre = T[:3, 3]
            
            # Fill J_p for this joint
            jacobian[0:3, i] = np.cross(z_pre, p_li - p_pre)
            # Fill J_o for this joint
            jacobian[3:6, i] = z_pre

        assert jacobian.shape == (6, 7)
        return jacobian

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        B = np.zeros((7, 7))
        # 1. Compute the jacobian at the centre of mass from second joint to last joint
        # 2. Compute forward kinematics at centre of mass from second to last joint
        # 3. Extract the J_p and J_o matrices from the Jacobian centre of mass matrices
        # 4. Calculate the inertia tensor using the rotation part of the FK centre of masses you have calculated
        # 5. Apply the the equation of getting B      

        B = np.zeros((7, 7))

        for i in range(1,len(joint_readings)+1):
            # The mass of the link
            m = self.mass[i-1]
            # Jacobian at the centre of mass
            J_cm = self.get_jacobian_centre_of_mass(joint_readings, i)
            J_P = J_cm[0:3, :]  # Position part of the Jacobian
            J_O = J_cm[3:6, :]  # Orientation part of the Jacobian
            # Rotation matrix
            R = self.forward_kinematics_centre_of_mass(joint_readings, i)[0:3, 0:3]
            I_p = np.diag(self.Ixyz[i-1])  # Inertia tensor for the parallel frame
            I_li = R @ I_p @ R.T  # Inertia tensor of the link
            # Calculate B
            B += m*(J_P.T @ J_P) + J_O.T @ I_li @ J_O
        
        return B

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7 
        # Some useful steps:
        # 1. Create a h_ijk matrix (a matrix containing the Christoffel symbols) and a C matrix.
        # 2. Compute the derivative of B components for the given configuration w.r.t. joint values q.
        # 3. Based on the previously obtained values, fill h_ijk.
        # 4. Based on h_ijk, fill C. Apply equations to compute C times qdot

        # Initialize:
        delta_q   = 10e-10
        n = len(joint_readings)
        C = np.zeros((n, n))
        h_ijk = np.zeros((n, n, n))  # Christoffel symbols
        B = self.get_B(joint_readings)
        joint_readings = np.array(joint_readings, dtype=float)
        
        for k in range(n):
            # q[k] + delta_q 
            dqk = np.copy(joint_readings)
            dqk[k] += delta_q
            for j in range(n):
                for i in range(n):
                    # q[i] + delta_q 
                    dqi = np.copy(joint_readings)
                    dqi[i] += delta_q
                    bij = self.get_B(dqk.tolist())
                    bjk = self.get_B(dqi.tolist())
                    dbij = (bij[i,j] - B[i,j])/delta_q
                    dbjk = (bjk[j,k] - B[j,k])/delta_q
                    h_ijk[i,j,k] = dbij - 0.5 * dbjk
            # Compute c_ij and add up as C       
            C += h_ijk[:,:,k] * joint_velocities[k]

        C = C @ joint_velocities

        assert isinstance(C, np.ndarray)
        assert C.shape == (7,)
        return C

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            g (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        # 1. Compute the Jacobian at CoM for all joints.
        # 2. Use the computed J_cm to fill the G matrix.
        # 3. Alternatvely, compute the P matrix and use it to fill the G matrix
        n = len(joint_readings)
        # intialise the g
        g = np.zeros((n))
        # g0^T
        g_0T = np.array([[0,0,-self.g]])
        

        # Method 1: Compute g with J_cm
        for i in range(n):
            m_li = self.mass[i]  # Mass of link i
            # Jacobian at CoM of link i
            J_cm_i = self.get_jacobian_centre_of_mass(joint_readings, i + 1)
            # Positional part of the Jacobian
            J_P_i = J_cm_i[0:3, :]  

            g -=  ((m_li * g_0T) @ J_P_i).squeeze()

        # Method 2: Compute g with derivative
        '''
        # initialise the delta
        delta_q = 10e-10
        joint_readings = np.array(joint_readings, dtype=float)

        for i in range(n):
            P_q_ = 0
            # q + delta
            q_ = np.copy(joint_readings)
            q_[i] = (q_[i] + delta_q).tolist()

            g_i = 0
            for j in range(n):
                m_li = self.mass[j]
                # Compute P(q+delta)
                p_li_ = self.forward_kinematics_centre_of_mass(q_.tolist(),j+1)[0:3,-1]
                P_q_ = - (m_li * g_0T @ p_li_)
                # Compute P(q)
                p_li = self.forward_kinematics_centre_of_mass(joint_readings.tolist(),j+1)[0:3,-1]
                P_q = - (m_li * g_0T @ p_li)

                temp = (P_q_ - P_q) / delta_q
                g_i = g_i + temp

            g[i] = g_i
        '''

        assert isinstance(g, np.ndarray)
        assert g.shape == (7,)
        return g