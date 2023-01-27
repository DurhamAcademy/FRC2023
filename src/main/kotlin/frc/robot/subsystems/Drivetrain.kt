package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.BLDriveMotorId
import frc.robot.Constants.BLTurnEncoderId
import frc.robot.Constants.BLTurnMotorId
import frc.robot.Constants.BRDriveMotorId
import frc.robot.Constants.BRTurnEncoderId
import frc.robot.Constants.BRTurnMotorId
import frc.robot.Constants.FLTurnEncoderId
import frc.robot.Constants.FLTurnMotorId
import frc.robot.Constants.FRTurnEncoderId
import frc.robot.Constants.FRTurnMotorId
import frc.robot.commands.DriveCommand
import frc.robot.controls.ControlScheme

class Drivetrain(
    val controlScheme: ControlScheme,
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
    val frontLeft = SwerveModule( // front right
        Constants.FLDriveMotorId,
        FLTurnMotorId,
        FLTurnEncoderId,
        "frontLeft",
        angleZero = 122.0,
        position = Translation2d(0.33, 0.33),
        controlScheme = controlScheme
    ) // FIXME: change postion to new drivebase measurements
    val frontRight = SwerveModule( // backleft
        Constants.FRDriveMotorId,
        FRTurnMotorId,
        FRTurnEncoderId,
        "frontRight",
        Translation2d(0.33, -0.33),
        angleZero = 73.0,
        controlScheme = controlScheme

    ) // FIXME: change postion to new drivebase measurements
    val backLeft = SwerveModule(
        BLDriveMotorId,
        BLTurnMotorId,
        BLTurnEncoderId,
        "backLeft",
        Translation2d(-0.33, 0.33),
        angleZero = 65.75,
        controlScheme = controlScheme

    ) // FIXME: change postion to new drivebase measurements
    val backRight = SwerveModule(
        BRDriveMotorId,
        BRTurnMotorId,
        BRTurnEncoderId,
        "backRight",
        Translation2d(-0.33, -0.33),
        angleZero = 154.5,
        controlScheme = controlScheme

    ) // FIXME: change postion to new drivebase measurements
    val modules = listOf(frontLeft, frontRight, backLeft, backRight)
    val kinematics = SwerveDriveKinematics(
        *modules.map { it.position }.toTypedArray()
    )

//    private val gyro: Gyro = ADXRS450_Gyro()

    // Odometry class for tracking robot pose
    var odometry = SwerveDriveOdometry(
        kinematics,
        Rotation2d(),//gyro.rotation2d,
        modules
            .map { it.position.toSwerveModulePosition() }
            .toTypedArray()
    )
    val Idrc = getTab("drivetrain")
    val power = Idrc.add("power", 0.0)

    override fun periodic() {
        // This method will be called once per scheduler run
        // Update the odometry in the periodic block
        gyroEntry.setDouble(0.0)
        odometry.update(
//            gyro.rotation2d,
            Rotation2d(),
            modules.map { it.position.toSwerveModulePosition() }.toTypedArray()
        )
    }

    val pose: Pose2d
        // Returns the currently-estimated pose of the robot
        get() = odometry.poseMeters

    // Resets the odometry to the specified pose
    fun resetOdometry(pose: Pose2d?) {
        odometry.resetPosition(
            Rotation2d(),//gyro.rotation2d,
            modules.map { it.position.toSwerveModulePosition() }.toTypedArray(),
            pose
        )
    }

    /**
     * Method to drive the robot using joystick info
     */
    fun drive(chassisSpeeds: ChassisSpeeds, fieldRelative: Boolean) {
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d())//gyro.rotation2d)
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
        this.frontLeft.resetEncoders()
        this.frontLeft.resetEncoders()
        backLeft.resetEncoders()
        backRight.resetEncoders()
    }

    /**
     * Zeroes the heading of the robot
     */
//    fun zeroHeading() = gyro.reset()

    /**
     * the heading of the robot.
     */
//    val heading: Double
//        get() = gyro.rotation2d.degrees

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
//    val turnRate: Double
//        get() = gyro.rate * if (Constants.gyroReversed) -1.0 else 1.0
    //fixme: add gyro stuff
}

private fun Translation2d.toSwerveModulePosition(): SwerveModulePosition = SwerveModulePosition(this.norm, this.angle)

