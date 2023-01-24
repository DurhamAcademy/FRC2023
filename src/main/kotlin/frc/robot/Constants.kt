// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object chassisConstants {
        var deadband = 0.15
        var kS = 0.667 / 12 //Test and revise if need
        var kV = 2.44 / 12 //Test and revise if need
        var kA = 0.27 / 12 //Test and revise if need
        var driveMotorGearRat = 6.54 / 1
        var turnMotorGearRat = 11.8 / 1
        val wheelDiameter = Units.inchesToMeters(4.0)
        var circumference = wheelDiameter * Math.PI
        var maxSpeedMPS = 6.5
        var maxTurnSpeed = 2.0

        //Distance between the front and back wheels
        const val wheelBase = 13.5625

        //Distance between left and right wheels
        const val trackWidth = 11.369
        val swerveKinematics = SwerveDriveKinematics(
            Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        )
    }

    object chassisSetUp {
        //Front Left Module
        const val fLeftDriveMotorPort = 10
        const val fLeftTurnMotorPort = 14
        const val isFrontLeftDriveMotorReverse = true
        const val isFrontLeftTurnMotorReverse = true
        const val fLeftAbsoluteEncoder = 6
        val frontLAngle = Units.radiansToDegrees(0.024654)
        const val frontLKP = 0.7
        const val frontLKI = 0.0
        const val frontLKD = 0.0

        //Front Right Module
        const val fRightDriveMotorPort = 11
        const val fRightTurnMotorPort = 15
        const val isFrontRightDriveMotorReverse = true
        const val isFrontRightTurnMotorReverse = true
        const val fRightAbsoluteEncoder = 7
        val frontRAngle = Units.radiansToDegrees(0.172576)
        const val frontRKP = 0.6
        const val frontRKI = 0.0
        const val frontRKD = 0.0

        //Back Right Module
        const val bRightDriveMotorPort = 12
        const val bRightTurnMotorPort = 16
        const val isBackRightDriveMotorReverse = true
        const val isBackRightTurnMotorReverse = true
        const val bRightAbsoluteEncoder = 8
        val backRAngle = Units.radiansToDegrees(0.221884)
        const val backRKP = 0.7
        const val backRKI = 0.0
        const val backRKD = 0.0

        //Back Left Module
        const val bLeftDriveMotorPort = 13
        const val bLeftTurnMotorPort = 17
        const val isBackLeftDriveMotorReverse = true
        const val isBackLeftTurnMotorReverse = true
        const val bLeftAbsoluteEncoder = 9
        val backLAngle = Units.radiansToDegrees(0.073961)
        const val backLKP = 0.6
        const val backLKI = 0.0
        const val backLKD = 0.0

        //Gyro
        const val invertedGyro = false
    }
}