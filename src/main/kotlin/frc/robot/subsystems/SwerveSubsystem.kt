// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.ctre.phoenix.sensors.WPI_Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.chassisConstants
import frc.robot.Constants.chassisSetUp
import org.photonvision.PhotonCamera
import java.io.IOException
import kotlin.math.IEEErem

class SwerveSubsystem : SubsystemBase() {
    private val frontLeft = SwerveModule(
        chassisSetUp.fLeftDriveMotorPort,
        chassisSetUp.isFrontLeftDriveMotorReverse,
        chassisSetUp.fLeftTurnMotorPort,
        chassisSetUp.isFrontLeftTurnMotorReverse,
        chassisSetUp.fLeftAbsoluteEncoder,
        chassisSetUp.frontLAngle,
        chassisSetUp.frontLKP,
        chassisSetUp.frontLKI,
        chassisSetUp.frontLKD
    )
    private val frontRight = SwerveModule(
        chassisSetUp.fRightDriveMotorPort,
        chassisSetUp.isFrontRightDriveMotorReverse,
        chassisSetUp.fRightTurnMotorPort,
        chassisSetUp.isFrontRightTurnMotorReverse,
        chassisSetUp.fRightAbsoluteEncoder,
        chassisSetUp.frontRAngle,
        chassisSetUp.frontRKP,
        chassisSetUp.frontRKI,
        chassisSetUp.frontRKD
    )
    private val backLeft = SwerveModule(
        chassisSetUp.bLeftDriveMotorPort,
        chassisSetUp.isBackLeftDriveMotorReverse,
        chassisSetUp.bLeftTurnMotorPort,
        chassisSetUp.isBackLeftTurnMotorReverse,
        chassisSetUp.bLeftAbsoluteEncoder,
        chassisSetUp.backLAngle,
        chassisSetUp.backLKP,
        chassisSetUp.backLKI,
        chassisSetUp.backLKD
    )
    private val backRight = SwerveModule(
        chassisSetUp.bRightDriveMotorPort,
        chassisSetUp.isBackRightDriveMotorReverse,
        chassisSetUp.bRightTurnMotorPort,
        chassisSetUp.isBackRightTurnMotorReverse,
        chassisSetUp.bRightAbsoluteEncoder,
        chassisSetUp.backRAngle,
        chassisSetUp.backRKP,
        chassisSetUp.backRKI,
        chassisSetUp.backRKD
    )
    private val gyro = WPI_Pigeon2(0)
    var swerveOdometry = SwerveDriveOdometry(chassisConstants.swerveKinematics, yaw, modulePositions)
    private lateinit var swerveModuleStates: Array<SwerveModuleState>

    init {
        Thread {
            try {
                Thread.sleep(1000)
                zeroHeading()
            } catch (_: Exception) {}
        }.start()
    }

    fun zeroHeading() {
        gyro.reset()
    }

    val heading: Double
        get() = gyro.angle.IEEErem(360.0)
    val rotation2d: Rotation2d
        get() = Rotation2d.fromDegrees(heading)
    val pose: Pose2d
        get() = swerveOdometry.poseMeters

    fun resetOdometry(pose: Pose2d?) {
        swerveOdometry.resetPosition(rotation2d, modulePositions, pose)
    }

    override fun periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Robot Heading", heading)
        SmartDashboard.putNumber("FrontL Angle: ", frontLeft.state.angle.degrees)
        SmartDashboard.putNumber("FrontR Angle: ", frontRight.state.angle.degrees)
        SmartDashboard.putNumber("BackL Angle: ", backLeft.state.angle.degrees)
        SmartDashboard.putNumber("BackR Angle: ", backRight.state.angle.degrees)
        SmartDashboard.putString("Robot Location", pose.translation.toString())
    }

    fun stopModules() {
        frontLeft.stop()
        frontRight.stop()
        backLeft.stop()
        backRight.stop()
    }

    fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean) {
        swerveModuleStates = chassisConstants.swerveKinematics.toSwerveModuleStates(
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
                translation!!.x,
                translation.y,
                rotation,
                yaw
            ) else ChassisSpeeds(
                translation!!.x,
                translation.y,
                rotation
            )
        )
        setModuleStates(swerveModuleStates)
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, chassisConstants.maxSpeedMPS)
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        backLeft.setDesiredState(desiredStates[2])
        backRight.setDesiredState(desiredStates[3])
    }

    val modulePositions: Array<SwerveModulePosition?>
        get() {
            val positions = arrayOfNulls<SwerveModulePosition>(4)
            positions[1-1] = frontLeft.position
            positions[2-1] = frontRight.position
            positions[3-1] = backLeft.position
            positions[4-1] = backRight.position
            return positions
        }
    val yaw: Rotation2d
        get() {
            val ypr = DoubleArray(3)
            gyro.getYawPitchRoll(ypr)
            return if (chassisSetUp.invertedGyro) Rotation2d.fromDegrees(360 - ypr[0]) else Rotation2d.fromDegrees(ypr[0])
        }

    companion object {
        var tracker = PhotonCamera("tracker")
        fun generateTrajectory(trajectory: String): Trajectory {
            var traj = Trajectory()
            try {
                val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory)
                traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
            } catch (ex: IOException) {
                DriverStation.reportError("Unable to open trajectory: $trajectory", ex.stackTrace)
            }
            return traj
        }
    }
}