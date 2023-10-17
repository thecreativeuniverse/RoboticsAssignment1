import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped
from .pf_base import PFLocaliserBase
import math
import rospy
from .util import rotateQuaternion, getHeading
import random as rn
from random import random, randint, choices
from time import time

class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 1  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.5  # Odometry model y axis (side-to-side) noise

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

        # Initialising self.particlecloud
        self.particlecloud = PoseArray()
        self.particlecloud.header.frame_id = "map"

        # Range should be set to however many particles we want
        new_particles = PoseArray()

        for i in range(self.poseArraySize):
            width = self.sensor_model.map_width * self.sensor_model.map_resolution
            height = self.sensor_model.map_height * self.sensor_model.map_resolution
            # This is generating the nose we need to add to the elements
            new_pose = Pose(position=Point((rn.uniform(0, width)),rn.uniform(0, height), 0),
                            orientation=Quaternion(0, 0, rn.uniform(0, math.pi * 2), rn.uniform(0, math.pi ** 2)))
            new_particles.poses.append(new_pose)

        self.particlecloud = new_particles

        return new_particles

    # Shouldn't be allowing partciles outside the map?
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        particle_array = []
        weight_array = []

        for particle in self.particlecloud.poses:
            particleWeight = self.sensor_model.get_weight(scan,particle)
            particle_array.append(particle)
            weight_array.append(particleWeight)

        tick_size = 1 / self.poseArraySize
        sum_of_weights = sum(weight_array)
        normalised_weights = [w / sum_of_weights for w in weight_array]

        current_threshold = rn.uniform(0, tick_size)
        cum_weight = 0

        kept_particles = []
        kept_weights = []

        for i in range(len(normalised_weights)):
            normalised_weight = normalised_weights[i]
            current_threshold += tick_size
            cum_weight += normalised_weight
            if current_threshold > cum_weight:
                continue
            kept_particles.append(particle_array[i])
            kept_weights.append(weight_array[i])

        # rn.shuffle(new_particles)
        print(f"REMOVED {self.poseArraySize - len(kept_particles)} PARTICLES")

        new_particles = kept_particles.copy()
        variance = 0.5

        if len(new_particles) == 0:
            self.initialise_particle_cloud(self.estimatedpose)
        else:
            while len(new_particles) < self.poseArraySize:
                random_pose_to_copy = choices(kept_particles, kept_weights, k=1)[0]
                new_pose = Pose(position=Point(random_pose_to_copy.position.x + rn.normalvariate(0, variance), random_pose_to_copy.position.y + rn.normalvariate(0, variance), 0),
                                orientation=Quaternion(0,0,random_pose_to_copy.orientation.z + rn.normalvariate(0, variance), random_pose_to_copy.orientation.w + rn.normalvariate(0, variance)))
                new_particles.append(new_pose)

            self.particlecloud.poses = new_particles
        # return s

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

        return Pose(orientation=Quaternion(avg_or_x, avg_or_y, avg_or_z, avg_or_w),
                    position=Point(avg_pos_x, avg_pos_y, avg_pos_z))
