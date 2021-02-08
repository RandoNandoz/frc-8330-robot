import math
import wpilib
import wpilib.drive
import wpilib.interfaces


def scale_input_xbone_triggers(port: int) -> float:
    xbone_controller = wpilib.XboxController(port)
    left_hand = wpilib.interfaces.GenericHID.Hand.kLeftHand
    right_hand = wpilib.interfaces.GenericHID.Hand.kRightHand
    trigger_value = xbone_controller.getTriggerAxis(right_hand) - (xbone_controller.getTriggerAxis(left_hand))
    if trigger_value < 0:
        return_value = -math.sqrt(abs(trigger_value))
    else:
        return_value = math.sqrt(abs(trigger_value))
    return return_value


class MyRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        self.right_motor_back = wpilib.PWMVictorSPX(0)
        self.right_motor_front = wpilib.PWMVictorSPX(1)
        self.left_motor_front = wpilib.PWMVictorSPX(2)
        self.left_motor_back = wpilib.PWMVictorSPX(3)

        self.right_speed_group = wpilib.SpeedControllerGroup(self.right_motor_back, self.right_motor_front)
        self.left_speed_group = wpilib.SpeedControllerGroup(self.left_motor_back, self.left_motor_front)

        self.drive = wpilib.drive.DifferentialDrive(self.left_speed_group, self.right_speed_group)

    def teleopPeriodic(self) -> None:
        xbone_controller = wpilib.XboxController(0)
        left = wpilib.interfaces.GenericHID.Hand.kLeftHand
        quick_turn = False
        if xbone_controller.getAButton():
            quick_turn = not quick_turn
        self.drive.curvatureDrive(scale_input_xbone_triggers(0), xbone_controller.getX(left), quick_turn)


if __name__ == "__main__":
    wpilib.run(MyRobot)
