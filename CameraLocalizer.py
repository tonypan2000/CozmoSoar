import process_markers
import numpy as np
import math
from threading import Thread


class CameraLocalizer:

    def __init__(self):
        self.localizer = process_markers.Localizer()
        self.camera_coord = None
        self.camera_angle = None
        self.cozmo_origin_location = 0
        self.cozmo_origin_rotation = 0
        self.change_of_bases_r_to_s = np.zeros([3, 3])
        self.change_of_bases_s_to_r = np.zeros([3, 3])
        self.update_change_of_basis()

    # start a thread for continuously processing markers
    def start(self):
        Thread(target=self._get_cam_pos, args=()).start()

    # get camera coordinates and angle
    def _get_cam_pos(self):
        while True:
            self.camera_coord, self.camera_angle = self.localizer.pose_from_camera()

    def update_change_of_basis(self):
        a = math.cos(self.cozmo_origin_rotation)
        b = math.sin(self.cozmo_origin_rotation)

        self.change_of_bases_r_to_s = np.transpose(np.array([[a, b, 0], [-b, a, 0], [0, 0, 1]]))
        self.change_of_bases_s_to_r = np.linalg.inv(self.change_of_bases_r_to_s)

    # pose: xyzrpy
    def recalculate_transform(self, cozmo_pose, world_pose):
        self.cozmo_origin_rotation = world_pose[5] - cozmo_pose[5]
        self.update_change_of_basis()

        cozmo_vec = np.transpose(np.array([cozmo_pose[0], cozmo_pose[1], cozmo_pose[2]]))
        origin_to_cozmo = np.transpose(np.matmul(self.change_of_bases_r_to_s, cozmo_vec))
        world_to_cozmo = np.array([world_pose[0], world_pose[1], world_pose[2]])

        self.cozmo_origin_location = world_to_cozmo - origin_to_cozmo

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


test = CameraLocalizer()
test.start()
