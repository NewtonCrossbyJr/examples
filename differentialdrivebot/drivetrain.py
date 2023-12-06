# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics, DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpilib import AnalogGyro, Encoder, MotorControllerGroup, PWMSparkMax

class Drivetrain:
    kMaxSpeed = 3.0  # meters per second
    kMaxAngularSpeed = 2 * 3.141592653589793  # one rotation per second

    kTrackWidth = 0.381 * 2  # meters
    kWheelRadius = 0.0508  # meters
    kEncoderResolution = 4096

    def __init__(self):
        self.m_leftLeader = PWMSparkMax(1)
        self.m_leftFollower = PWMSparkMax(2)
        self.m_rightLeader = PWMSparkMax(3)
        self.m_rightFollower = PWMSparkMax(4)

        self.m_leftEncoder = Encoder(0, 1)
        self.m_rightEncoder = Encoder(2, 3)

        self.m_leftGroup = MotorControllerGroup(self.m_leftLeader, self.m_leftFollower)
        self.m_rightGroup = MotorControllerGroup(self.m_rightLeader, self.m_rightFollower)

        self.m_gyro = AnalogGyro(0)

        self.m_leftPIDController = PIDController(1, 0, 0)
        self.m_rightPIDController = PIDController(1, 0, 0)

        self.m_kinematics = DifferentialDriveKinematics(Drivetrain.kTrackWidth)

        self.m_feedforward = SimpleMotorFeedforwardMeters(1, 3)

        self.m_gyro.reset()

        self.m_rightGroup.setInverted(True)

        self.m_leftEncoder.setDistancePerPulse(
            2 * 3.141592653589793 * Drivetrain.kWheelRadius / Drivetrain.kEncoderResolution
        )
        self.m_rightEncoder.setDistancePerPulse(
            2 * 3.141592653589793 * Drivetrain.kWheelRadius / Drivetrain.kEncoderResolution
        )

        self.m_leftEncoder.reset()
        self.m_rightEncoder.reset()

        self.m_odometry = DifferentialDriveOdometry(
            self.m_gyro.getRotation2d(),
            self.m_leftEncoder.getDistance(),
            self.m_rightEncoder.getDistance(),
        )

    def setSpeeds(self, speeds):
        leftFeedforward = self.m_feedforward.calculate(speeds.left)
        rightFeedforward = self.m_feedforward.calculate(speeds.right)

        leftOutput = self.m_leftPIDController.calculate(
            self.m_leftEncoder.getRate(), speeds.left
        )
        rightOutput = self.m_rightPIDController.calculate(
            self.m_rightEncoder.getRate(), speeds.right
        )
        self.m_leftGroup.setVoltage(leftOutput + leftFeedforward)
        self.m_rightGroup.setVoltage(rightOutput + rightFeedforward)

    def drive(self, xSpeed, rot):
        wheelSpeeds: DifferentialDriveWheelSpeeds = self.m_kinematics.toWheelSpeeds(ChassisSpeeds(xSpeed, 0.0, rot))
        self.setSpeeds(wheelSpeeds)

    def updateOdometry(self):
        self.m_odometry.update(
            self.m_gyro.getRotation2d(),
            self.m_leftEncoder.getDistance(),
            self.m_rightEncoder.getDistance(),
        )
