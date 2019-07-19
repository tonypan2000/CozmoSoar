from threading import Thread
import process_markers
import numpy as np
import math


class CameraLocalizer:

    def __init__(self):
        # Spin up a thread to periodically update the transform
        Thread(target=self._transform, args=()).start()
        self.localizer = process_markers.Localizer()
        self.cozmo_origin_location = 0
        self.cozmo_origin_rotation = 0
        self.change_of_bases_r_to_s = np.zeros([3, 3])
        self.change_of_bases_s_to_r = np.zeros([3, 3])
        self.update_change_of_basis()

    def update_change_of_basis(self):
        a = math.cos(self.cozmo_origin_rotation)
        b = math.sin(self.cozmo_origin_rotation)

        self.change_of_bases_r_to_s = np.transpose(np.array([[a, b, 0], [-b, a, 0], [0, 0, 1]]))
        self.change_of_bases_s_to_r = np.inv(self.change_of_bases_r_to_s)

        return
    # transform
    def transform(self):
        while True:
            camera_coord, camera_angle = self.localizer.pose_from_camera()
            #cozmo_pose = self.get_cozmo_pose(self, camera_coord, camera_angle)
            #world_pose = self.get_world_pose(self, cozmo_pose)
            #self.update_robot_pose(self, world_pose)

    # pose: xyzrpy
    def update_robot_pose(self, cozmo_pose, world_pose):
        self.cozmo_origin_rotation = world_pose[5] - cozmo_pose[5]
        self.update_change_of_basis()

        cozmo_vec = np.transpose(np.array([cozmo_pose[0], cozmo_pose[1], cozmo_pose[2]]))
        origin_to_cozmo = np.transpose(np.matmul(self.change_of_bases_r_to_s, cozmo_vec))
        world_to_cozmo = np.array([world_pose[0], world_pose[1], world_pose[2]])

        self.cozmo_origin_location = world_to_cozmo - origin_to_cozmo

        return

    # cozmo_pose: xyzrpy
    def get_world_pose(self, cozmo_pose):
        cozmo_vec = np.transpose(np.array([cozmo_pose[0], cozmo_pose[1], cozmo_pose[2]]))
        world_rot = self.cozmo_origin_rotation + cozmo_pose[5]

        world_vec = np.matmul(self.change_of_bases_r_to_s, cozmo_vec)
        world_vec += self.cozmo_origin_location

        return world_vec, world_rot

    # world_pose: xyzrpy
    def get_cozmo_pose(self, world_pose):
        world_vec = np.transpose(np.array([world_pose[0], world_pose[1], world_pose[2]]))
        cozmo_rot = world_pose[5] - self.cozmo_origin_rotation

        cozmo_vec = np.matmul(self.change_of_bases_s_to_r, world_vec)
        cozmo_vec -= self.cozmo_origin_location

        return cozmo_vec, cozmo_rot
