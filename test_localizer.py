import cozmo
import time


def cozmo_program(robot: cozmo.robot.Robot):
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
            position = [robot.pose.position.x / 1000, robot.pose.position.y / 1000,
                        robot.pose.position.z / 1000, 0, 0, yaw]
            camera.recalculate_transform(position)
            cube_yaw = cube[0].pose.rotation.angle_z.radians
            cube_position = [cube[0].pose.position.x / 1000, cube[0].pose.position.y / 1000,
                             cube[0].pose.position.z / 1000, 0, 0, cube_yaw]
            print("Cozmo in Cozmo coordinates", position)
            print("Cube in Cozmo coordinates", cube_position)
            cube_pose_world = camera.get_world_pose(cube_position)
            print("Cube in camera position (calculated): ", cube_pose_world)


cozmo.run_program(cozmo_program)
