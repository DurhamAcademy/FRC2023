package frc.robot6502.subsystems

import com.ctre.phoenix.sensors.WPI_PigeonIMU
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot6502.Constants
import frc.robot6502.Constants.BLDriveMotorId
import frc.robot6502.Constants.BLTurnEncoderId
import frc.robot6502.Constants.BLTurnMotorId
import frc.robot6502.Constants.BRDriveMotorId
import frc.robot6502.Constants.BRTurnEncoderId
import frc.robot6502.Constants.BRTurnMotorId
import frc.robot6502.Constants.FLDriveMotorId
import frc.robot6502.Constants.FLTurnEncoderId
import frc.robot6502.Constants.FLTurnMotorId
import frc.robot6502.Constants.FRDriveMotorId
import frc.robot6502.Constants.FRTurnEncoderId
import frc.robot6502.Constants.FRTurnMotorId
import frc.robot6502.commands.DriveCommand
import frc.robot6502.controls.ControlScheme

class Drivetrain(
    controlScheme: ControlScheme,
) : SubsystemBase() {
    init {
        defaultCommand = DriveCommand(this, controlScheme)
    }

    private val swerveTab = getTab("Swerve Diagnostics")

    //private PowerDistribution PDP = new PowerDistribution();
    private val xSpeedEntry = swerveTab.add("xBox xSpeed", 0)
        .entry
    private val ySpeedEntry = swerveTab.add("xBox ySpeed", 0)
        .entry
    private val rotEntry = swerveTab.add("xBox rot", 0)
        .entry
    private val gyroEntry = swerveTab.add("Gyro Heading", 0)
        .entry
    val frontLeft = SwerveModule(
        FLDriveMotorId,
        FLTurnMotorId,
        FLTurnEncoderId,
        "frontLeft",
        Translation2d(0.33, 0.33),
    ) // FIXME: change postion to new drivebase measurements
    val frontRight = SwerveModule(
        FRDriveMotorId,
        FRTurnMotorId,
        FRTurnEncoderId,
        "frontRight",
        Translation2d(0.33, -0.33),
    ) // FIXME: change postion to new drivebase measurements
    val backLeft = SwerveModule(
        BLDriveMotorId,
        BLTurnMotorId,
        BLTurnEncoderId,
        "backLeft",
        Translation2d(-0.33, 0.33),
    ) // FIXME: change postion to new drivebase measurements
    val backRight = SwerveModule(
        BRDriveMotorId,
        BRTurnMotorId,
        BRTurnEncoderId,
        "backRight",
        Translation2d(-0.33, -0.33),
    ) // FIXME: change postion to new drivebase measurements
    val modules = listOf(frontLeft, frontRight, backLeft, backRight)
    val kinematics = SwerveDriveKinematics(
        *modules.map { it.position }.toTypedArray()
    )

    private val gyro: Gyro = WPI_PigeonIMU(0)

    // Odometry class for tracking robot pose
    var odometry = SwerveDriveOdometry(
        kinematics,
        gyro.rotation2d,
        modules
            .map { it.position.toSwerveModulePosition() }
            .toTypedArray()
    )

    override fun periodic() {
        // This method will be called once per scheduler run
        // Update the odometry in the periodic block
        gyroEntry.setDouble(gyro.angle)
        odometry.update(
            gyro.rotation2d,
            modules.map { it.position.toSwerveModulePosition() }.toTypedArray()
        )
    }

    val pose: Pose2d
        // Returns the currently-estimated pose of the robot
        get() = odometry.poseMeters

    // Resets the odometry to the specified pose
    fun resetOdometry(pose: Pose2d?) {
        odometry.resetPosition(
            gyro.rotation2d,
            modules.map { it.position.toSwerveModulePosition() }.toTypedArray(),
            pose
        )
    }

    /**
     * Method to drive the robot using joystick info
     */
    fun drive(chassisSpeeds: ChassisSpeeds, fieldRelative: Boolean) {
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.rotation2d)
            else chassisSpeeds
        )
//        SwerveDriveKinematics.desaturateWheelSpeeds(
//            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
//        )
        swerveModuleStates.forEachIndexed { i, swerveModuleState ->
            modules[i].setpoint = swerveModuleState
        }
        // Telemetry
        xSpeedEntry.setDouble(chassisSpeeds.vxMetersPerSecond)
        ySpeedEntry.setDouble(chassisSpeeds.vyMetersPerSecond)
        rotEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond)
        //fixme: move the following to swerve module periodic
        modules.forEachIndexed { i, module ->
            module.stateEntry.setDouble(swerveModuleStates[i].speedMetersPerSecond)
        }
    }

    /**
     *
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState?>) {
//        SwerveDriveKinematics.desaturateWheelSpeeds(
//            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
//        )
        // fixme: find max speed

        modules.forEachIndexed { i, module ->
            module.setpoint = SwerveModuleState(
                desiredStates[i]?.speedMetersPerSecond ?: 0.0,
                desiredStates[i]?.angle ?: Rotation2d()
            )
        }
    }

    /**
     *  Resets the drive encoders to currently read a position of 0
     */
    fun resetEncoders() {
        frontLeft.resetEncoders()
        backLeft.resetEncoders()
        frontRight.resetEncoders()
        backRight.resetEncoders()
    }

    /**
     * Zeroes the heading of the robot
     */
    fun zeroHeading() = gyro.reset()

    /**
     * the heading of the robot.
     */
    val heading: Double
        get() = gyro.rotation2d.degrees

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    val turnRate: Double
        get() = gyro.rate * if (Constants.gyroReversed) -1.0 else 1.0
    //fixme: add gyro stuff
}

private fun Translation2d.toSwerveModulePosition(): SwerveModulePosition = SwerveModulePosition(this.norm, this.angle)

