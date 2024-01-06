#!/usr/bin/env python3
import numpy as np
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from Robot_Dynamics.iiwa14DynKDL import Iiwa14DynamicKDL
from sensor_msgs.msg import JointState
from scipy.linalg import inv
import matplotlib.pyplot as plt
import threading

class iiwa14TrajectoryPlanning(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('iiwa14_traj', anonymous=True)

        # Initialize an object of class Iiwa14DynamicKDL
        self.Iiwa14KDL = Iiwa14DynamicKDL()
        # Create trajectory publisher
        self.traj_pub = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                        queue_size=7)
        # Create joint states subscriber
        self.joints_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.callback_JointStates)

        self.accelerations = []  # List to store acceleration arrays
        self.times = []  # List to store time stamps

    def run(self):
        """
        This function is the main run function of the class. When called, it runs question 5 by calling the q5() function,
        and run plot_accelerations() after 20 seconds.
        """
        print("run q5")
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(2.0)
        traj = self.q5()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
        self.traj_pub.publish(traj)

        # Schedule the plotting function to run after 20 seconds    
        threading.Timer(20, self.plot_accelerations).start()

    def q5(self):
        """ 
        Methods are called to load messages from bag file by calling 'load_targets()',
        create and return the trajectory of the robot.
        """

        # Load the messages form the rosbag
        target_p, target_v, target_a, target_e = self.load_targets()

        # Create a trajectory message and publish to get the robot to move along target trajectory
        traj = JointTrajectory()
        t = 0
        dt = 5
        for i in range(target_p.shape[1]):
            traj_point = JointTrajectoryPoint()
            traj_point.effort = target_e[i]
            traj_point.positions = target_p[:, i]
            traj_point.velocities = target_v[:, i]
            traj_point.accelerations = target_a[:, i]
            t = t + dt
            traj_point.time_from_start.secs = t
            traj.points.append(traj_point)

        assert isinstance(traj, JointTrajectory)
        return traj

    def load_targets(self):
        """
        This function loads messages from the 'cw3q5.bag' file.
        Returns:
            target_p: The target joint values for the 3 target points position.
            target_v: The target joint values for the 3 target points velocitie.
            target_a: The target joint values for the 3 target points acceleration.
            target_e: Effort applied on the joints
        """
        # Defining ros package path
        rospack = rospkg.RosPack()
        path = rospack.get_path('cw3q5')

        # Initialize arrays for target joint positions, velocities, and accelerations
        target_p = np.zeros((7, 3))
        target_v = np.zeros((7, 3))
        target_a = np.zeros((7, 3))
        target_e = []

        # Load path for selected question
        bag = rosbag.Bag(path + '/bag/cw3q5.bag')

        # Extract the joint data iteratively  
        for topic, msg, t in bag.read_messages():
            for i, point in enumerate(msg.points):
                target_p[:, i] = point.positions
                target_v[:, i] = point.velocities
                target_a[:, i] = point.accelerations
                target_e.append(point.effort)
        # Close the bag
        bag.close()

        return target_p, target_v, target_a, target_e
    
    def callback_JointStates(self, msg):
        '''
        This function extracts data from message 'JointState' to calculate the accelerations,
        and store data for plotting.
        '''
        # Extracts the joint states
        self.p = msg.position
        self.v = msg.velocity
        self.effort = msg.effort
        # Calculate the accelerations based on forward dynamics
        self.a = self.get_a()
        self.time = rospy.get_time()


        # Store the data for plotting
        self.accelerations.append(self.a)
        self.times.append(self.time)

    def get_a(self):
        '''
        This function calculates the accelerations based on forward dynamics
        by calling functions from the KDL class.
        '''
        # Initialize acceleration matrix
        a = np.ones((7,1))
        # Calculate components of equations of motion
        B = np.array(self.Iiwa14KDL.get_B(self.p))
        C_times_qdot = np.array(self.Iiwa14KDL.get_C_times_qdot(self.p, self.v))
        G = np.array(self.Iiwa14KDL.get_G(self.p))
        # Forward dynamics
        a = inv(B) @ (self.effort - C_times_qdot - G)

        return a
    
    def plot_accelerations(self):
        # Convert the stored lists to numpy arrays for easier slicing
        accelerations = np.array(self.accelerations)
        times = np.array(self.times)
        
        # Plotting each joint acceleration
        plt.figure(figsize=(10, 8))
        for i in range(7):
            plt.plot(times, accelerations[:, i], label=f'Joint {i+1}')
        
        plt.title('Joint Accelerations over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (rad/s²)')
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    try:
        iiwa_planner = iiwa14TrajectoryPlanning()
        iiwa_planner.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
