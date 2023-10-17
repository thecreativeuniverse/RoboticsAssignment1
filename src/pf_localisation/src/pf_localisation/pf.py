import math
import random as rn

import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point, PoseWithCovarianceStamped

from .pf_base import PFLocaliserBase


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Sensor model parameters
        self.set_num_predicted_readings(20)

        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 1  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1  # Odometry model x-axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.5  # Odometry model y-axis (side-to-side) noise

        # ----- Set particle array parameters
        self.pose_array_size = 100

        init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
        init_pose_pub.publish(self.estimatedpose)

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
        # Generate initial particle cloud: standard normal distribution around initialposition
        new_particles.poses = [self.generate_pose(pose=initialpose.pose.pose, variance=1) for _ in range(self.pose_array_size)]
        self.particlecloud = new_particles

        return new_particles

    # Shouldn't be allowing particles outside the map?
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        # Get current poses
        initial_particles = self.particlecloud.poses
        # Add 20% more particles in uniform random locations; this will allow us to find a more accurate estimation of
        # the robot's position in the event the current estimation is completely wrong
        initial_particles += [self.generate_pose() for _ in range(round(len(initial_particles) * 0.2))]

        # Calculate the weights of all particles and append them to an array, then calculate an average
        particles_weights = [self.sensor_model.get_weight(scan, particle) for particle in initial_particles]
        sum_of_weights = sum(particles_weights)
        average_weight = sum_of_weights / len(particles_weights)

        # we vary the amount of particles based on how certain the algorithm is of the robot's position
        particles_to_keep = round((100 / average_weight ** 0.5) - (average_weight / 10))
        new_predicted_readings = round(10 * average_weight)
        self.set_pose_array_size(particles_to_keep)
        self.set_num_predicted_readings(new_predicted_readings)

        # if the algorithm needs to remove particles it will remove the particles with the lowest weight
        while len(particles_weights) > particles_to_keep:
            index = particles_weights.index((min(particles_weights)))
            del (particles_weights[index])
            del (initial_particles[index])

        # if the algorithm needs to generate particles it generates 50 particles and places the particle with the highest weight
        while len(particles_weights) < particles_to_keep:
            new_poses = []
            for i in range(50):
                new_pose = self.generate_pose()
                new_weight = self.sensor_model.get_weight(scan, new_pose)
                new_poses.append((new_pose, new_weight))
            new_poses = sorted(new_poses, key=lambda x: x[1])

            initial_particles.append(new_poses[-1][0])
            particles_weights.append(new_poses[-1][1])

        sum_of_weights = sum(particles_weights)
        average_weight = sum_of_weights / len(particles_weights)
        variance = (1 / (average_weight - 0.8))

        particles_weights = [w / sum_of_weights for w in particles_weights]
        particles_kept = []  # This is the S
        current_cum_weight = particles_weights[0]
        cum_weights = [current_cum_weight]

        m = self.pose_array_size
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
                i += 1

            particles_kept.append(self.generate_pose(pose=initial_particles[i], variance=variance))
            current_threshold += tick_size

        self._update_poses(particles_kept)

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

        # Calculating closest particles
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
            temp = math.sqrt((particle.position.x - avg_pos_x) ** 2 + (particle.position.y - avg_pos_y) ** 2 + (
                    particle.position.z - avg_pos_z) ** 2 + (particle.orientation.x - avg_or_x) ** 2 + (
                                     particle.orientation.y - avg_or_y) ** 2 + (
                                     particle.orientation.z - avg_or_z) ** 2 + (
                                     particle.orientation.w - avg_or_w) ** 2)
            distances.append((particle, temp))

        sorted_distance = sorted(distances, key=lambda x: x[1])
        # When wanting half of the particles use [:50] at the end of the list name

        halved = length // 2

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

    def set_pose_array_size(self, new_array_size):
        self.pose_array_size = new_array_size

    def set_num_predicted_readings(self, num_predicted_readings):
        self.NUMBER_PREDICTED_READINGS = num_predicted_readings

    def _update_poses(self, new_poses):
        self.particlecloud.poses = new_poses

    def generate_pose(self, pose=None, variance=0.0):
        width = self.sensor_model.map_width * self.sensor_model.map_resolution
        height = self.sensor_model.map_height * self.sensor_model.map_resolution
        if pose is not None:
            return Pose(position=Point(pose.position.x + rn.normalvariate(0, variance),
                                       pose.position.y + rn.normalvariate(0, variance),
                                       0),
                        orientation=Quaternion(0, 0,
                                               pose.orientation.z + rn.normalvariate(0, variance),
                                               pose.orientation.w + rn.normalvariate(0, variance)))
        else:
            return Pose(position=Point((rn.uniform(0, width)), rn.uniform(0, height), 0),
                        orientation=Quaternion(0, 0, rn.uniform(0, math.pi * 2), rn.uniform(0, math.pi ** 2)))
