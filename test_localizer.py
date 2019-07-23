import cozmo


class TestBot:
    def __init__(self, robot: cozmo.robot.Robot):
        self.bot = robot
        self.bot.enable_device_imu(True, True, True)
        from CameraLocalizer import CameraLocalizer
        self.camera = CameraLocalizer()

    async def run(self):
        self.camera.start()
        while True:
            pitch, yaw, roll = self.bot.device_gyro.euler_angles
            print("Yaw: ", yaw)
            pos = self.bot.pose.position
            print(pos)


async def cozmo_program(robot: cozmo.robot.Robot):
    bot = TestBot(robot)
    await bot.run()

cozmo.run_program(cozmo_program)
