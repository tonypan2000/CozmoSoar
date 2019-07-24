import cozmo
import time
from threading import Thread


# class TestBot:
#     def __init__(self, robot: cozmo.robot.Robot):
#         self.bot = robot
#         self.bot.enable_device_imu(True, True, True)
#         from CameraLocalizer import CameraLocalizer
#         self.camera = CameraLocalizer()
#
#     # start a thread to get cozmo pose
#     def start(self):
#         Thread(target=self._run, args=()).start()
#
#     def _run(self):
#         while True:
#             pitch, yaw, roll = self.bot.device_gyro.euler_angles
#             # print("Yaw: ", yaw)
#             pos = self.bot.pose.position
#             # print(pos)


def cozmo_program(robot: cozmo.robot.Robot):
    # bot = TestBot(robot)
    # bot.start()
    # print("robot started")
    robot.enable_device_imu(True, True, True)
    from CameraLocalizer import CameraLocalizer
    camera = CameraLocalizer()
    camera.start()
    print("camera initialized")
    time.sleep(5)
    cube = None
    while not cube:
        cube = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
        if cube:
            print("found a cube")
            _, yaw, _ = robot.device_gyro.euler_angles
            pos = robot.pose.position
            position = [pos.x / 1000, pos.y / 1000, pos.z / 1000, 0, 0, yaw]

            cube_yaw = cube[0].pose.rotation.angle_z.radians

            cube_pos = cube[0].pose.position

            cube_position = [cube_pos.x / 1000, cube_pos.y / 1000, cube_pos.z / 1000, 0, 0, cube_yaw]
            print("Cozmo in Cozmo coordinates", position)
            print("Cube in Cozmo coordinates", cube_position)
            world_vec, world_rot = camera.get_world_pose(cube_position, position)
            print("Cube in camera position (calculated): ", world_vec)
            print("Cube in camera rotation (calculated): ", world_rot)


cozmo.run_program(cozmo_program)
