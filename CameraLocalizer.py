import process_markers


class CameraLocalizer:

    def __init__(self):
        self.localizer = process_markers.Localizer()

    # transform
    def transform(self):
        while True:
            camera_coord, camera_angle = self.localizer.pose_from_camera()
            #cozmo_pose = self.get_cozmo_pose(self, camera_coord, camera_angle)
            #world_pose = self.get_world_pose(self, cozmo_pose)
            #self.update_robot_pose(self, world_pose)

    # pose: xyzrpy
    def update_robot_pose(self, pose):

        pass

    # cozmo_pose: xyzrpy
    def get_world_pose(self, cozmo_pose):

        pass

    # world_pose: xyzrpy
    def get_cozmo_pose(self, camera_coord, camera_angle):
        # tvecs
        # angle

        pass


test = CameraLocalizer()
test.transform()
