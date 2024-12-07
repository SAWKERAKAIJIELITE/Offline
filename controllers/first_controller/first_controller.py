"""first_controller controller."""

# from controller import Robot, Motor, DistanceSensor

import inspect
import math
from controller import Robot

# print(inspect.getmodule(Robot).__file__)

E_PUCK_WHEEL_RADIUS = 0.02
E_PUCK_RADIUS = 0.026
E_PUCK_MAX_VELOCITY = 6.28


class RobotController(Robot):

    CLOCK_WISE = 1
    COUNTER_CLOCK_WISE = 0

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        # self.another_tg_pos = 0

        self.left_motor_sensor = self.getDevice("left wheel sensor")
        self.right_motor_sensor = self.getDevice("right wheel sensor")

        self.left_motor_sensor.enable(self.time_step)
        self.right_motor_sensor.enable(self.time_step)

        # They will not be enabled until one time step
        self.step(self.time_step)

    def run_motors(self, distance: float, left_velocity: float = E_PUCK_MAX_VELOCITY, right_velocity: float = E_PUCK_MAX_VELOCITY):
        """
        Method that will run two motors for specific velocity for specific distance
        the ratio between distance and wheel radius is the amount that should the wheel rotate

        because:
            distance = 2 * pi * wheel_radius * rho
                            distance
            arc_length = ------------ = 2 * pi * rho
                            wheel_radius

        Args:
            distance (float):
            left_velocity (float, optional): Defaults to E_PUCK_MAX_VELOCITY.
            right_velocity (float, optional): Defaults to E_PUCK_MAX_VELOCITY.
        """

        arc_length = distance / E_PUCK_WHEEL_RADIUS

        right_old_position = self.right_motor_sensor.getValue()
        left_old_position = self.left_motor_sensor.getValue()

        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

        right_amount = self.right_motor_sensor.getValue() - right_old_position
        left_amount = self.left_motor_sensor.getValue() - left_old_position

        while (abs(right_amount) <= arc_length or abs(left_amount) <= arc_length):
            # print(f'{arc_length=} ')

            m = self.right_motor_sensor.getValue()
            n = self.left_motor_sensor.getValue()

            self.step(self.time_step)

            step_distance_right = self.right_motor_sensor.getValue() - m
            step_distance_left = self.left_motor_sensor.getValue() - n

            # print(f'{step_distance_right=}')
            # print(f'{step_distance_left=}')

            right_amount = self.right_motor_sensor.getValue() - right_old_position
            left_amount = self.left_motor_sensor.getValue() - left_old_position

            # print(f'{right_amount=}')
            # print(f'{left_amount=}')

        new_left_velocity = (
            arc_length -
            abs(left_amount)
        ) * left_velocity / step_distance_left

        new_right_velocity = (
            arc_length -
            abs(right_amount)
        ) * right_velocity / step_distance_right

        if new_left_velocity * left_velocity > 0:
            new_left_velocity *= -1

        if new_right_velocity * right_velocity > 0:
            new_right_velocity *= -1

        self.left_motor.setVelocity(new_left_velocity)
        self.right_motor.setVelocity(new_right_velocity)

        # print(f'{new_left_velocity=}')
        # print(f'{new_right_velocity=}')

        m = self.right_motor_sensor.getValue()
        n = self.left_motor_sensor.getValue()

        self.step(self.time_step)

        step_distance_right = self.right_motor_sensor.getValue()-m
        step_distance_left = self.left_motor_sensor.getValue()-n

        # print(f'{step_distance_right=}')
        # print(f'{step_distance_left=}')

        # print(f'{self.right_motor_sensor.getValue() - right_old_position=}')
        # print(f'{self.left_motor_sensor.getValue() - right_old_position=}')

        # reset motors velocities to zero
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def turn_around_robot(self, angle: float, direction: int = CLOCK_WISE, velocity: float = E_PUCK_MAX_VELOCITY):
        """
        Args:
            angle (float): in radian
            direction (int, optional): CLOCK_WISE or COUNTER_CLOCK_WISE . Defaults to CLOCK_WISE.
            velocity (float, optional): Defaults to E_PUCK_MAX_VELOCITY.
        """
        if direction == self.CLOCK_WISE:
            self.run_motors(angle * E_PUCK_RADIUS, velocity, -velocity)

        elif direction == self.COUNTER_CLOCK_WISE:
            self.run_motors(angle * E_PUCK_RADIUS, -velocity, velocity)

    def loop_square(self):
        while self.step(self.time_step) != -1:
            self.run_motors(0.25, 6, 6)
            self.turn_around_robot(math.pi / 2, velocity=1)
            # break


r = RobotController()
r.loop_square()

# r.left_motor.setPosition(10)
# r.right_motor.setPosition(-10)

# while r.step(r.time_step) != -1:
#     print(f'{r.left_motor.getPositionSensor().getValue()=}')
#     print(f'{r.right_motor.getPositionSensor().getValue()=}')
#     if r.left_motor.getPositionSensor().getValue() >= 9:
#         r.left_motor.setPosition(0)
#         r.right_motor.setPosition(-20)
#     print(f'{r.left_motor.getTargetPosition()=}')
#     print(f'{r.left_motor.getMinPosition()=}')
#     print(f'{r.left_motor.getMaxPosition()=}')
#     print(f'{r.left_motor.getBrake()=}')

# print(dir(Robot))
# المسافة هي عدد مرات تكرار محيط عجلة الروبوت أو بمعنى آخر عدد اللفات حول عجلة نصف قطره واحد و بالتالي يمكن معرفة عدد اللفات حول عجلة الروبوت من خلال تقسيمها على نصف قطر عجلة الروبوت

# ToDo: higher Velocity
