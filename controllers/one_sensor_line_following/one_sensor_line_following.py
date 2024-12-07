"""one_sensor_line_following controller."""


# from controller import Robot, Motor, DistanceSensor

from controller import Robot

E_PUCK_MAX_VELOCITY = 6.28


class RobotController(Robot):

    def __init__(self):

        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        self.sensor = self.getDevice("line follow sensor")
        self.sensor.enable(self.time_step)

        self.step(self.time_step)

    def line_follow(self, velocity: float = E_PUCK_MAX_VELOCITY):

        value = self.sensor.getValue()

        if value < 200:
            self.left_motor.setVelocity(velocity)
            self.right_motor.setVelocity(velocity / 2)
        elif value > 200:
            self.left_motor.setVelocity(velocity / 2)
            self.right_motor.setVelocity(velocity)
        else:
            self.left_motor.setVelocity(velocity)
            self.right_motor.setVelocity(velocity)

        print(value)

    def loop(self):
        while self.step(self.time_step) != -1:
            self.line_follow()

    def print_sensor_value(self):
        while self.step(self.time_step) != -1:
            print(self.sensor.getValue())


r = RobotController()
r.loop()
# r.print_sensor_value()
