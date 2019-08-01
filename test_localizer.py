import cozmo
import time


def cozmo_program(robot: cozmo.robot.Robot):
    robot.enable_device_imu(True, True, True)
    from CameraLocalizer import CameraLocalizer
    camera = CameraLocalizer()
    camera.start()
    while not camera.ready:
        time.sleep(0.5)
    print("camera initialized")
    yaw = robot.pose.rotation.angle_z.radians
    position = [robot.pose.position.x / 1000, robot.pose.position.y / 1000,
                robot.pose.position.z / 1000, 0, 0, yaw]
    print("Cozmo in Cozmo coordinates", position)
    camera.recalculate_transform(position)
    cube = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=10)
    if cube:
        cube_yaw = cube[0].pose.rotation.angle_z.radians
        cube_position = [cube[0].pose.position.x / 1000, cube[0].pose.position.y / 1000,
                         cube[0].pose.position.z / 1000, 0, 0, cube_yaw]
        print("Cube in Cozmo coordinates", cube_position)
        cube_pose_world = camera.get_world_pose(position, cube_position)
        print("Cube in camera position (calculated): ", cube_pose_world)
        # cube_pose_cozmo = camera.get_cozmo_pose(camera.camera_cube_position)
        # print("Cube in Cozmo Coordinates (calculated)", cube_pose_cozmo)


cozmo.run_program(cozmo_program)
