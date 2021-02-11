"""
Code to drive the team 8330 robot.

Essentially, right trigger forwards, left backwards.
Left stick steers.
A button allow the robot to spin in place. X allows for full power.
"""

# Imports
import wpilib
import wpilib.drive
import wpilib.interfaces


def scale_input_xbone_triggers(controller: wpilib.XboxController, factor: float) -> float:
    """
    :param controller: The Xbox controller object.
    Function to scale the input from the Xbox One controller's triggers. Right trigger returns a positive
    value from 0 to 1, left trigger returns values from -1 to 0. Currently, the system works linearly.

    :param factor: The factor to scale the power by.
    :return: Returns a value from 0 to 1, from the inputs of the right and left bumpers.
    """

    # Define the left and right hands.
    left_hand = wpilib.interfaces.GenericHID.Hand.kLeftHand
    right_hand = wpilib.interfaces.GenericHID.Hand.kRightHand
    # The trigger value is the right bumpers value minus the left's.
    trigger_value = controller.getTriggerAxis(right_hand) - (controller.getTriggerAxis(left_hand))
    # If the triggers' total is less than zero, we need to take the square root of it,
    # then multiply it by the factor, and then negate it.
    if trigger_value < 0:
        return_value = -abs(trigger_value) * factor
    # Otherwise, just assign the sqrt times the factor.
    else:
        return_value = abs(trigger_value) * factor
    return return_value


def vib_xbone_to_scale(controller: wpilib.XboxController, val: float):
    """ Vibrate the controller from values -1 to 1."""
    if val < 0:
        controller.setRumble(wpilib.interfaces.GenericHID.RumbleType.kLeftRumble, abs(val))
    else:
        controller.setRumble(wpilib.interfaces.GenericHID.RumbleType.kRightRumble, abs(val))


# noinspection PyAttributeOutsideInit
class MyRobot(wpilib.TimedRobot):
    """
    This is the main code for the robot. It has robotInit(), which runs on robot start, and it just simply does these:

    1. Initialize the motors on PWM pins 0 to 3. These are used to drive the robot forwards and backwards.
    2. Make the speed groups, as the robot is driven with a pair of gearboxes, both connected to 2 motors each.
    3. Create the drive class from the two speed controller groups.

    teleopPeriodic() does different things however.
    It runs constantly when the robot is running and just checks for the
    Xbox controller's left stick movement, checks for the A and X button presses,
    and turns the robot, and changes functionality.
    """

    # This function runs on robot initialization - like the name says.
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

        # Quick turn logic, essentially, when the A button is pressed, we allow the robot to quickly turn on it's
        # X axis. Start with quick_turn on False,
        # as the teleopPeriodic function is run constantly during robot operation,
        # we just check for the A button press, and then negate the value.
        quick_turn = False
        # We make the turn value here.
        turn_value = xbone_controller.getX(left)
        # The scale factor here.
        scale_factor = 1 / 3
        if xbone_controller.getAButton():
            quick_turn = not quick_turn
            # If the A button is pressed, give scaling factor of the steering power to avoid overloading the robot.
            turn_value = turn_value * scale_factor

        # Allow for full power to the robot systems with the X button.
        if xbone_controller.getXButton():
            # Essentially same system as the quick turn.
            scale_factor = 1
        # We use curvature drive, with the upper helper function for the Xbox controller,
        # and turn with the left hand joystick.
        self.drive.curvatureDrive(scale_input_xbone_triggers(xbone_controller, scale_factor), turn_value,
                                  quick_turn)
        vib_xbone_to_scale(xbone_controller, scale_input_xbone_triggers(xbone_controller, 1))


if __name__ == "__main__":
    wpilib.run(MyRobot)
