import process_markers
import numpy as np
import math
from threading import Thread

# Reverse x axis
# transposes in functions may need fixing
class CameraLocalizer:

    def __init__(self):
        self.localizer = process_markers.Localizer()
        self.world_position = np.zeros([6])
        self.camera_cube_position = np.zeros([6])
        self.cozmo_origin_location = 0
        self.cozmo_origin_rotation = 0
        self.change_of_bases_r_to_s = np.zeros([3, 3])
        self.change_of_bases_s_to_r = np.zeros([3, 3])
        # TODO: update world_position here so that we can update transform and calculate change of basis immediately
        # self.recalculate_transform(np.array([0,0,0,0,0,0]), self.world_position)
        # maybe need to get cozmo_pose not zeros
        # self.update_change_of_basis()

    # start a thread for continuously processing markers
    def start(self):
        Thread(target=self._get_cam_pos, args=()).start()

    # get camera coordinates and angle
    def _get_cam_pos(self):
        while True:
            camera_cozmo_coord, camera_cozmo_ang, camera_cube_coord, camera_cube_ang = self.localizer.pose_from_camera()
            if camera_cozmo_coord is not None:
                self.world_position = np.array([camera_cozmo_coord[0], camera_cozmo_coord[1], camera_cozmo_coord[2],
                                                camera_cozmo_ang[0], camera_cozmo_ang[1], camera_cozmo_ang[2]])
                # print("world pos: ", self.world_position)
                self.camera_cube_position = np.array([camera_cube_coord[0], camera_cube_coord[1], camera_cube_coord[2],
                                                camera_cube_ang[0], camera_cube_ang[1], camera_cube_ang[2]])

    # updates: change of basis matricies
    def update_change_of_basis(self):
        a = math.cos(self.cozmo_origin_rotation)
        b = math.sin(self.cozmo_origin_rotation)

        self.change_of_bases_r_to_s = np.transpose(np.array([[a, b, 0], [-b, a, 0], [0, 0, 1]]))
        self.change_of_bases_s_to_r = np.linalg.inv(self.change_of_bases_r_to_s)

    # pose: xyzrpy
    # pass in: (cozmo pose to cozmo, cozmo pose to world)
    # updates: origin location and rotation
    def recalculate_transform(self, cozmo_pose, world_pose):
        self.cozmo_origin_rotation = world_pose[5] - cozmo_pose[5]
        self.update_change_of_basis()

        cozmo_vec = np.transpose(np.array([cozmo_pose[0], cozmo_pose[1], cozmo_pose[2]]))
        origin_to_cozmo = np.transpose(np.matmul(self.change_of_bases_r_to_s, cozmo_vec))
        world_to_cozmo = np.array([world_pose[0], world_pose[1], world_pose[2]])

        self.cozmo_origin_location = world_to_cozmo - origin_to_cozmo

    # cozmo_pose: xyzrpy
    # pass in: (object position to cozmo, cozmo position to cozmo)
    def get_world_pose(self, object_pose, cozmo_pose):
        # need to update cozmo in camera coordinates (world_position)
        print("Cozmo in camera coordinates", self.world_position)
        self.recalculate_transform(cozmo_pose, self.world_position)
        object_vec = np.transpose(np.array([object_pose[0], object_pose[1], object_pose[2]]))
        world_rot = self.cozmo_origin_rotation + object_pose[5]

        world_vec = np.matmul(self.change_of_bases_r_to_s, object_vec)
        world_vec += self.cozmo_origin_location
        print("cube in camera position (actual): ", self.camera_cube_position)
        return world_vec, world_rot

    # world_pose: xyzrpy
    # pass in: (object position in world, cozmo position to cozmo)
    def get_cozmo_pose(self, world_pose, cozmo_pose):
        self.recalculate_transform(cozmo_pose, self.world_position)
        world_vec = np.transpose(np.array([world_pose[0], world_pose[1], world_pose[2]]))
        cozmo_rot = world_pose[5] - self.cozmo_origin_rotation

        cozmo_vec = np.matmul(self.change_of_bases_s_to_r, world_vec)
        cozmo_vec -= self.cozmo_origin_location

        return cozmo_vec, cozmo_rot


# test = CameraLocalizer()
# test.start()
