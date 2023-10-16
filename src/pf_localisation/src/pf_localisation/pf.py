import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from .pf_base import PFLocaliserBase
import math
import rospy
from .util import rotateQuaternion, getHeading
from random import random, randint
import random as rn
from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.5  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.5  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.5  # Odometry model y axis (side-to-side) noise

        self.poseArraySize = 200
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
        # map_range = self.sensor_model.calc_map_range(self.estimatedpose.pose.pose.position.x, self.estimatedpose.pose.pose.position.y,
        #                              getHeading(self.estimatedpose.pose.pose.orientation))
        map_range = 30
        initialised_poses = [Pose(orientation=Quaternion(0, 0, (random()*2)-1, (random()*2)-1),
                                  position=Point((random() * map_range), (random() * map_range), 0)) for i in range(self.poseArraySize)]
        self.particlecloud.poses = initialised_poses
        self.pub.publish(self.particlecloud)



        return self.particlecloud

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        # Initialise the post position set
        s = []
        # Getting the weights aka line 4
        weights =[]
        for particle in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, particle)
            print(f"paritcle pos={particle.position.x} {particle.position.y}\tparticleweight={weight}")
            weights.append(weight)

        print(weights)
        print("=" * 100)

        summed_weights = sum(weights)
        weights = [weight / summed_weights for weight in weights]
        cum_weights = [weights[0]]
        for i in range(1,len(weights)):
            cum_weights.append(cum_weights[-1] + weights[i])

        # Initialising the cumulative weights array

        # Generate CDF (line 2)
        # for i in range(len(weights)):
        #     cum_weight = cum_weights[i-1] + weights[i]
        #     cum_weights.append(cum_weight)

        # for i in range(len(cum_weights)):
        #     cum_weights.append(cum_weights[i]/cum_weights[-1])
        # M is a number of particles
        m_inv = self.poseArraySize ** -1
        # u is the initial threshold
        u = random() * m_inv

        print("CUM WEIGHTS")
        print(cum_weights)

        i = 1

        map_range = 30
        for j in range(self.poseArraySize):
            if u <= cum_weights[j]: # doesn't like this particle. make a new one based on last accepted no
                s += [self.particlecloud.poses[j]]
            u += m_inv
        while len(s) < self.poseArraySize:
            new_pose = Pose(orientation=Quaternion(0, 0, (random()*2)-1, (random()*2)-1))
            new_pose.position = s[randint(0, len(s) - 1)].position
            new_pose.position.x += (random() * self.ODOM_TRANSLATION_NOISE) - self.ODOM_TRANSLATION_NOISE / 2
            new_pose.position.y += (random() * self.ODOM_TRANSLATION_NOISE) - self.ODOM_TRANSLATION_NOISE / 2
            new_pose.orientation.z += (random() * self.ODOM_ROTATION_NOISE) - self.ODOM_ROTATION_NOISE / 2
            new_pose.orientation.w += (random() * self.ODOM_ROTATION_NOISE) - self.ODOM_ROTATION_NOISE / 2
            s += [new_pose]


        self.particlecloud.poses = s

        #return s

        # TODO Add in number of particles at random locations at some point to factor in kidnapped robot problem

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).Pose()
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        position_x = 0
        position_y = 0
        position_z = 0

        orientation_x = 0
        orientation_y = 0
        orientation_z = 0
        orientation_w = 0

        for particle in self.particlecloud.poses:
            position_x += particle.position.x
            position_y += particle.position.y
            position_z += particle.position.z

            orientation_x += particle.orientation.x
            orientation_y += particle.orientation.y
            orientation_z += particle.orientation.z
            orientation_w += particle.orientation.w

        length = len(self.particlecloud.poses)
        avg_pos_x = position_x / length
        avg_pos_y = position_y / length
        avg_pos_z = 0

        avg_or_x = 0
        avg_or_y = 0
        avg_or_z = orientation_z / length
        avg_or_w = orientation_w / length

        return Pose(orientation=Quaternion(avg_or_x, avg_or_y, avg_or_z, avg_or_w), position=Point(avg_pos_x, avg_pos_y, avg_pos_z))
