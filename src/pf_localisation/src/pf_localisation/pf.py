		from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
import math
import rospy
from . util import rotateQuaternion, getHeading
from random import random
from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        print(PFLocaliser)
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0  # Odometry model y axis (side-to-side) noise


        self.poseArraySize = 100
        self.pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=10)



    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

        self.particlecloud = PoseArray()
        self.particlecloud.header.frame_id = "map"
        #poses = [Pose() for i in range(self.poseArraySize)]
        #for pose in poses:
        #    pose.orientation = Quaternion
        initialised_poses = [Pose(orientation=Quaternion(random(),random(),random(),random()),position=Point(random()*10,random()*10,0)) for i in range(self.poseArraySize)]
        self.particlecloud.poses = initialised_poses
        print(self.particlecloud.poses[69])
        self.pub.publish(self.particlecloud)
        return self.particlecloud

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
         
         #weights = [self.sensor_model.getweight(scan, particle) for particle in self.particlecloud.poses]
         
         #S = EMPTY LIST
         #cum_weights = np.zeros(len(weights))
         #cum_weights[0] = weights[0]
         #for i in range(2, self.particlecloud.poses):
         #	cum_weights[i] = cum_weights[i-1] + weights[i]
         # LINE 4 OF ALGORITHM
         
         	
         
         
         #Add in number of particles at random locations at some point to factor in kidnapped robot problem
         
         	
         
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        pass
