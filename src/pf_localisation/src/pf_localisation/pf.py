import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from .pf_base import PFLocaliserBase
import math
import rospy
from .util import rotateQuaternion, getHeading
from random import random
from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        print(PFLocaliser)

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0  # Odometry model y axis (side-to-side) noise

        self.poseArraySize = 100
        self.pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=10, latch=True)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(orientation=Quaternion(0, 0, 0, 0), position=Point(0, 0, 0))
        self.init_pose = initial_pose

        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
        self.init_pose_pub.publish(self.estimatedpose)

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
        # poses = [Pose() for i in range(self.poseArraySize)]
        # for pose in poses:
        #    pose.orientation = Quaternion
        initialised_poses = [Pose(orientation=Quaternion(random(), random(), random(), random()),
                                  position=Point(random() * 100, random() * 100, 0)) for i in range(self.poseArraySize)]
        self.particlecloud.poses = initialised_poses
        self.pub.publish(self.particlecloud)


        print("====INITIAL POSE====")
        print(initialpose)

        return self.particlecloud

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        print("====UPDATE PARTICLE CLOUD====")
        # Initialise the post position set
        s = []
        # Getting the weights aka line 4
        weights = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses]
        print(weights)

        # Initialising the cumulative weights array
        cum_weights = np.zeros(len(weights))
        cum_weights[0] = weights[0]

        # Generate CDF (line 2)
        for i in range(2, self.poseArraySize):
            cum_weights[i] = cum_weights[i - 1] + weights[i]

        # M is a number of particles
        m_inv = self.poseArraySize ** -1
        # u is the initial threshold
        u = random() * m_inv

        i = 1
        for j in range(self.poseArraySize):
            while u > i:
                i = i + 1
            s = s + [self.particlecloud.poses[i]]
            u = u + m_inv

        return s

        # TODO Add in number of particles at random locations at some point to factor in kidnapped robot problem

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
        location = 0
        orientation = 0

        print("==================ESTIMATE POSE=================")
        #for i in self.particlecloud.poses:
         #   location += self.particlecloud.poses[i]

        return Pose() #TODO
