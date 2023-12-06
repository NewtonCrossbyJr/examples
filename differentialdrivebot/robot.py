# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from drivetrain import Drivetrain
from wpimath.filter import SlewRateLimiter

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.m_controller = wpilib.XboxController(0)
        self.m_drive = Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.m_speedLimiter = SlewRateLimiter(3)
        self.m_rotLimiter = SlewRateLimiter(3)

    def autonomousPeriodic(self):
        self.teleopPeriodic()
        self.m_drive.updateOdometry()

    def teleopPeriodic(self):
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = -self.m_speedLimiter.calculate(self.m_controller.getLeftY()) * Drivetrain.kMaxSpeed

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = -self.m_rotLimiter.calculate(self.m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed

        self.m_drive.drive(xSpeed, rot)

if __name__ == '__main__':
    wpilib.run(Robot)