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
            initial_pose_position = initialpose.pose.pose.position
            initial_pose_orientation = initialpose.pose.pose.orientation

            new_pose = Pose(position=Point(initial_pose_position.x + rn.normalvariate(0, 1), initial_pose_position.y + rn.normalvariate(0, 1), 0),
                            orientation=Quaternion(0, 0, initial_pose_orientation.z + rn.normalvariate(0, 1),
                                                   initial_pose_orientation.w + rn.normalvariate(0, 1)))
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

        initial_particles = self.particlecloud.poses
        old_particles = self.particlecloud.poses
        particles_weights = [self.sensor_model.get_weight(scan, particle) for particle in old_particles]

        sum_of_weights = sum(particles_weights)

        average_weight = sum_of_weights / len(particles_weights)

        particles_to_keep = int(round((100 / average_weight ** 0.5) - (average_weight / 10)))
        self.poseArraySize = particles_to_keep
        self.NUMBER_PREDICTED_READINGS = int(round(10 * average_weight))

        print("posearraysize:",self.poseArraySize)

        while len(particles_weights) > particles_to_keep:
            print("current length (dec)", len(particles_weights))
            index = particles_weights.index((min(particles_weights)))
            del(particles_weights[index])
            del(old_particles[index])

        width = self.sensor_model.map_width * self.sensor_model.map_resolution
        height = self.sensor_model.map_height * self.sensor_model.map_resolution
        while len(particles_weights) < particles_to_keep:
            print("current length (inc)", len(particles_weights))
            new_poses = []
            for i in range(50):
                new_pose = Pose(position=Point((rn.uniform(0, width)), rn.uniform(0, height), 0),
                            orientation=Quaternion(0, 0, rn.uniform(0, math.pi * 2), rn.uniform(0, math.pi ** 2)))
                new_weight = self.sensor_model.get_weight(scan, new_pose)
                new_poses.append((new_pose, new_weight))
            new_poses = sorted(new_poses, key=lambda x: x[1])

            old_particles.append(new_poses[-1][0])
            particles_weights.append(new_poses[-1][1])

        sum_of_weights = sum(particles_weights)
        average_weight = sum_of_weights / len(particles_weights)
        variance = (1 / (average_weight - 0.8))

        particles_weights = [w / sum_of_weights for w in particles_weights]
        particles_kept = []  # This is the S
        current_cum_weight = particles_weights[0]
        cum_weights = [current_cum_weight]


        m = self.poseArraySize
        for i in range(1, m):
            cum_weights.append(cum_weights[i - 1] + particles_weights[i])

        tick_size = 1 / m
        current_threshold = rn.uniform(0, tick_size)

        # Safety in case it picks 0, shouldn't really enter this
        while current_threshold == 0:
            current_threshold = rn.uniform(0, tick_size)

        i = 0
        for j in range(m):
            while current_threshold > cum_weights[i]:
                # print(f"incrementing i {i}")
                i += 1

            particles_kept.append(Pose(position=Point(initial_particles[i].position.x + rn.normalvariate(0, variance),
                                                      initial_particles[i].position.y + rn.normalvariate(0, variance),
                                                      0),
                                       orientation=Quaternion(0, 0,
                                                              initial_particles[i].orientation.z + rn.normalvariate(0,
                                                                                                                    variance),
                                                              initial_particles[i].orientation.w + rn.normalvariate(0,
                                                                                                                    variance))))
            current_threshold += tick_size

        self.particlecloud.poses = particles_kept

        # # Initialization
        # particle_array = []
        # weight_array = []
        #
        # # Getting the weights of each particle and storing it in weight array
        # for particle in self.particlecloud.poses:
        #     particleWeight = self.sensor_model.get_weight(scan,particle)
        #     particle_array.append(particle)
        #     weight_array.append(particleWeight)
        #
        # # This is out M
        # tick_size = 1 / (self.poseArraySize)
        #
        # # Normalizing the weights so that they are between 0 and 1
        # sum_of_weights = sum(weight_array)
        # # normalised_weights = [w / sum_of_weights for w in weight_array]
        # cumulative_normalised_weights = [weight_array[0] / sum_of_weights]
        # for i in range(1, len(weight_array)):
        #     cumulative_normalised_weights.append(cumulative_normalised_weights[-1] + (weight_array[i] / sum_of_weights))
        #
        # # selecting what u1 will be (line 4)
        # current_threshold = random() * tick_size
        #
        # # Initialising c1 to w1
        #
        # kept_particles = []
        # kept_weights = []
        #
        # i = 0
        #
        # # Actual drawing of samples and filtering out bad weights
        # for j in range(len(cumulative_normalised_weights)):
        #     while current_threshold > cumulative_normalised_weights[i]:
        #         i += 1
        #     # print(f"threshold {current_threshold} cum weight {cumulative_normalised_weights[i]}")
        #     kept_particles.append(particle_array[i])
        #     kept_weights.append(weight_array[i])
        #
        #     # normalised_weight = normalised_weights[i]
        #     # current_threshold += tick_size
        #     # cum_weight += normalised_weight
        #     # if current_threshold > cum_weight:
        #     #     continue
        #     # kept_particles.append(particle_array[i])
        #     # kept_weights.append(weight_array[i])
        #
        # # rn.shuffle(new_particles)
        # print(f"REMOVED {self.poseArraySize - len(kept_particles)} PARTICLES")
        #
        # new_particles = kept_particles.copy()
        # variance = 0.5
        #
        # if len(new_particles) == 0:
        #     print("=" * 1000)
        #     self.initialise_particle_cloud(self.estimatedpose)
        # else:
        #     while len(new_particles) < self.poseArraySize:
        #         random_pose_to_copy = choices(kept_particles, kept_weights, k=1)[0]
        #         new_pose = Pose(position=Point(random_pose_to_copy.position.x + rn.normalvariate(0, variance), random_pose_to_copy.position.y + rn.normalvariate(0, variance), 0),
        #                         orientation=Quaternion(0,0,random_pose_to_copy.orientation.z + rn.normalvariate(0, variance), random_pose_to_copy.orientation.w + rn.normalvariate(0, variance)))
        #         new_particles.append(new_pose)
        #
        #     self.particlecloud.poses = new_particles
        # # return s
        #
        # print(f"len new poses {len(self.particlecloud.poses)}")
        #
        # # TODO Add in number of particles at random locations at some point to factor in kidnapped robot problem

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

        ## Calculating closest particles
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

        # Finding 50% closest particles
        distances = []
        for particle in self.particlecloud.poses:
            temp = math.sqrt((particle.position.x - avg_pos_x)**2 + (particle.position.y - avg_pos_y)**2 +(particle.position.z - avg_pos_z)**2 + (particle.orientation.x - avg_or_x)**2 + (particle.orientation.y - avg_or_y)**2 + (particle.orientation.z - avg_or_z)**2 +(particle.orientation.w - avg_or_w)**2)
            distances.append((particle, temp))

        
        sorted_distance = sorted(distances, key = lambda x: x[1])
        # When wanting half of the particles use [:50] at the end of the list name
        #print(sorted_distance[:50])

        halved = length//2

        sorted_particles = sorted_distance[:halved]

        position_x = 0
        position_y = 0
        position_z = 0

        orientation_x = 0
        orientation_y = 0
        orientation_z = 0
        orientation_w = 0

        for particle in sorted_particles:
            position_x += particle[0].position.x
            position_y += particle[0].position.y
            position_z += particle[0].position.z

            orientation_x += particle[0].orientation.x
            orientation_y += particle[0].orientation.y
            orientation_z += particle[0].orientation.z
            orientation_w += particle[0].orientation.w

        avg_pos_x = position_x / halved
        avg_pos_y = position_y / halved
        avg_pos_z = 0

        avg_or_x = 0
        avg_or_y = 0
        avg_or_z = orientation_z / halved
        avg_or_w = orientation_w / halved

        return Pose(orientation=Quaternion(avg_or_x, avg_or_y, avg_or_z, avg_or_w),
                    position=Point(avg_pos_x, avg_pos_y, avg_pos_z))
