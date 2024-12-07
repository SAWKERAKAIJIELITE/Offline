"""wall_following controller."""


from controller import Robot


E_PUCK_WHEEL_RADIUS = 0.02
E_PUCK_RADIUS = 0.026
E_PUCK_MAX_VELOCITY = 6.28


def range_convertor(s_start: float, s_end: float, d_start: float, d_end: float, value: float) -> float:
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ratio = abs((value - s_start) / (s_end - s_start))

    if d_start > d_end:
        return d_start - (abs(d_end - d_start) * ratio)

    return d_start + (abs(d_end - d_start) * ratio)


class RobotController(Robot):
    SENSOR_BOUND = (200, 400)
    RANGE = 50

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # initializing distance sensors
        # NOTE: all of these sensors are proximity sensor so they give bigger values as they go closer to the detected object (e.g. a wall).
        self.front_sensor = self.getDevice("ps0")
        self.front_sensor.enable(self.time_step)

        self.corner_sensor = self.getDevice("ps1")
        self.corner_sensor.enable(self.time_step)

        self.right_sensor = self.getDevice("ps2")
        self.right_sensor.enable(self.time_step)

        self.step(self.time_step)

    def steering(self, steering: float, velocity: float = E_PUCK_MAX_VELOCITY):
        """
        A function that is responsible for the steering functionality for the motor
        Steering value:
            - from -100 to 0 will turn left
            - from 0 to 100 will turn right
            - if equals 100 will turn the robot around itself to the right
            - if equals -100 will turn the robot around itself to the left
            - if equals 50 will turn the robot around right wheel to the right
            - if equals -50 will turn the robot around left wheel to the left
        """
        # right_velocity = velocity if steering < 0 else range_convertor(
        #     0,
        #     100,
        #     -velocity,
        #     velocity,
        #     steering
        # )
        if steering > 0:
            left_velocity = velocity
            right_velocity = range_convertor(
                -self.RANGE,
                self.RANGE,
                velocity,
                2 * velocity,
                steering
            )

        elif steering < 0:
            right_velocity = velocity
            left_velocity = range_convertor(
                -self.RANGE,
                self.RANGE,
                velocity,
                2 * velocity,
                steering
            )
        else:
            right_velocity = left_velocity = velocity
        # left_velocity = velocity if steering > 0 else range_convertor(
        #     -100,
        #     0,
        #     -velocity,
        #     velocity,
        #     steering
        # )
        print(f'{right_velocity=}')
        print(f'{left_velocity=}')

        self.right_motor.setVelocity(right_velocity)
        self.left_motor.setVelocity(left_velocity)

    def wall_following_step(self, velocity: float = E_PUCK_MAX_VELOCITY):
        """
        The function that is responsible for doing one step of the wall following (without looping)
        """

        right_value = self.right_sensor.getValue()
        front_value = self.front_sensor.getValue()
        corner_value = self.corner_sensor.getValue()

        self.left_motor.setVelocity(velocity)
        self.right_motor.setVelocity(velocity)

        print(f'{right_value=}')
        print(f'{corner_value=}')

        # checking front wall condition
        if front_value > 80 or corner_value > 80:
            self.left_motor.setVelocity(-2)
            self.right_motor.setVelocity(2)

        else:
            # run steering according the right sensor value
            new_var = range_convertor(
                # the minimum value of the sensor
                self.SENSOR_BOUND[0],
                # the maximum value of the sensor
                self.SENSOR_BOUND[1],
                # the maximum value of the steering to make it smooth
                -self.RANGE,
                # the minimum value of the steering to make it smooth
                self.RANGE,
                # the value of the sensor which we want to map
                right_value
            )
            print(f'{new_var=}')

            self.steering(
                new_var,
                velocity
            )

    def wall_following(self):
        """
        The function that is responsible for the wall fallowing functionality (with looping) 
        """
        while self.step(self.time_step) != -1:
            self.wall_following_step()


r = RobotController()
r.wall_following()
# print(r.right_sensor.getMinValue())
# print(r.right_sensor.getMaxValue())
# print(f'{r.right_sensor.getValue()= }')


# ToDo: E_PUCK_MAX_VELOCITY / spiral
