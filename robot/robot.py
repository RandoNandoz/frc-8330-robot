"""
Code to drive the team 8330 robot.
"""

# Imports
import math
import wpilib
import wpilib.drive
import wpilib.interfaces


def scale_input_xbone_triggers(port: int, factor: float) -> float:
    """
    Function to scale the input from the Xbox One controller's triggers. Right bumper returns a positive
    value from 0 to 1, back bumper returns a negative from 0 to 1. Both triggers negate another.
    :param port: The port the Xbox controller is on.
    :param factor: The factor to scale the power by.
    :return: Returns a value from 0 to 1, from the inputs of the right and left bumpers.
    """

    # Create the Xbox controller.
    xbone_controller = wpilib.XboxController(port)
    # Define the left and right hands.
    left_hand = wpilib.interfaces.GenericHID.Hand.kLeftHand
    right_hand = wpilib.interfaces.GenericHID.Hand.kRightHand
    # The trigger value is the right bumpers value minus the left's.
    trigger_value = xbone_controller.getTriggerAxis(right_hand) - (xbone_controller.getTriggerAxis(left_hand))
    # If the triggers' total is less than zero, we need to take the square root of it,
    # then multiply it by the factor, and then negate it.
    if trigger_value < 0:
        return_value = -math.sqrt(abs(trigger_value) * factor)
    # Otherwise, just assign the sqrt times the factor.
    else:
        return_value = math.sqrt(abs(trigger_value) * factor)
    return return_value


# noinspection PyAttributeOutsideInit
class MyRobot(wpilib.TimedRobot):

    # This function runs on robot initialization - like the name says.s
    def robotInit(self) -> None:
        # Create the motors, on pins 0 to 4.
        self.right_motor_back = wpilib.PWMVictorSPX(0)
        self.right_motor_front = wpilib.PWMVictorSPX(1)
        self.left_motor_front = wpilib.PWMVictorSPX(2)
        self.left_motor_back = wpilib.PWMVictorSPX(3)

        # Because we're using a gearbox (don't remember which), we need to make speed groups.
        self.right_speed_group = wpilib.SpeedControllerGroup(self.right_motor_back, self.right_motor_front)
        self.left_speed_group = wpilib.SpeedControllerGroup(self.left_motor_back, self.left_motor_front)

        # Differential drive from the two speed controllers.
        self.drive = wpilib.drive.DifferentialDrive(self.left_speed_group, self.right_speed_group)

    # The teleopPeriodic function runs when the robot is running.
    def teleopPeriodic(self) -> None:
        # Create the Xbox controller on port zero.
        xbone_controller = wpilib.XboxController(0)
        # Assign left a value.
        left = wpilib.interfaces.GenericHID.Hand.kLeftHand

        # Quick turn logic, essentially, when the "a" button is pressed, we allow the robot to quickly turn on it's
        # X axis. Start with quick_turn on False,
        # as the teleopPeriodic function is run constantly during robot operation,
        # we just check for the A button press, and then negate the value.
        quick_turn = False
        if xbone_controller.getAButton():
            quick_turn = not quick_turn

        # We use curvature drive, with the upper helper function for the Xbox controller,
        # and turn with the left hand joystick.
        self.drive.curvatureDrive(scale_input_xbone_triggers(0, 1 / 3), xbone_controller.getX(left), quick_turn)


if __name__ == "__main__":
    wpilib.run(MyRobot)
