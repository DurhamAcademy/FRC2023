package frc.robot.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
import frc.robot.PhotonCameraWrapper
import frc.robot.commands.DriveCommand
import frc.robot.controls.ControlScheme
import java.lang.Math.PI

class Drivetrain(
    val controlScheme: ControlScheme,
    val cameraWrappers: List<PhotonCameraWrapper>,
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
        angleZero = Constants.FLZeroAngle,
        location = Translation2d(
            Constants.MODULE_DISTANCE_X / 2,
            Constants.MODULE_DISTANCE_Y / 2
        )
    )
    val frontRight = SwerveModule( // backleft
        Constants.FRDriveMotorId,
        FRTurnMotorId,
        FRTurnEncoderId,
        "frontRight",
        angleZero = Constants.FRZeroAngle,
        location = Translation2d(
            Constants.MODULE_DISTANCE_X / 2,
            -Constants.MODULE_DISTANCE_Y / 2
        )
    )
    val backLeft = SwerveModule(
        BLDriveMotorId,
        BLTurnMotorId,
        BLTurnEncoderId,
        "backLeft",
        Translation2d(
            -Constants.MODULE_DISTANCE_X / 2,
            Constants.MODULE_DISTANCE_Y / 2
        ),
        angleZero = Constants.BLZeroAngle,
    ) // FIXME: change postion to new drivebase measurements
    val backRight = SwerveModule(
        BRDriveMotorId,
        BRTurnMotorId,
        BRTurnEncoderId,
        "backRight",

        Translation2d(-Constants.MODULE_DISTANCE_X / 2, -Constants.MODULE_DISTANCE_Y / 2),
        angleZero = Constants.BRZeroAngle,
    )
    val modules = listOf(frontLeft, frontRight, backLeft, backRight)
    val kinematics = SwerveDriveKinematics(
        *modules.map { it.location }.toTypedArray()
    )

    private val gyro = Pigeon2(54, "rio").apply {
        configFactoryDefault()
    }


    /**
     * The odometry object for tracking robot pose
     * This is used to get the robot's position on the field using the motors.
     * When the motors move, the odometry object calculates where the robot
     * should be according to the motors. What the motors report will, on its
     * own, not be accurate enough and slowly drift from the actual position.
     *
     * To correct for the drift, we use the camera's pose estimators to
     * correct the odometry object's position estimate. This is done in the
     * periodic method.
     * @author A1cD
     * @see periodic
     * @see SwerveDriveOdometry
     */
    var odometry = SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.yaw),
        modules
            .map { it.swerveModulePosition }
            .toTypedArray()
    )

    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.fromDegrees(gyro.yaw),
        modules.map {
            it.swerveModulePosition
        }.toTypedArray(),
        Pose2d()
    )

    val Idrc = getTab("drivetrain")
    val power = Idrc.add("power", 0.0)

    /**
     * The periodic method is run roughly every 20ms. This is where we update
     * any values that are constantly changing, such as the robot's position,
     * the robot's heading, and the robot's speed
     *
     * @author A1cD
     */
    override fun periodic() {
        // This method will be called once per scheduler run

        // we need to update odometry and the pose estimator with the current
        // position of the robot.
        odometry.update(
            Rotation2d.fromDegrees(gyro.yaw),
            modules.map { it.swerveModulePosition }.toTypedArray()
        )
        // and the pose estimator:
        poseEstimator.update(
            Rotation2d.fromDegrees(gyro.yaw),
            modules.map { it.swerveModulePosition }.toTypedArray()
        )

//        poseEstimator
        // we want to get the estimated position from each camera's pose
        // estimator and add it to the drivetrain's pose estimator. This helps
        // us get a more accurate position estimate
        cameraWrappers.forEach { cameraWrapper ->
            val pose = cameraWrapper.getEstimatedGlobalPose(poseEstimator.estimatedPosition)
            pose.ifPresent { estimatedRobotPose -> // if the camera sees a target,
                // and make sure we aren't adding a measurement that is too new
                // add the vision measurement to the drivetrain pose estimator
                poseEstimator.addVisionMeasurement(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds
                )
            }
        }

        SmartDashboard.putNumber("gyroangle", gyro.yaw)
        SmartDashboard.putNumber("uptime", gyro.upTime.toDouble())
        gyroEntry.setDouble(gyro.yaw)
    }

    /**
     * The pose of the robot
     */
    val pose: Pose2d
        // Returns the currently-estimated pose of the robot
        get() = poseEstimator.estimatedPosition

    /**
     * Resets the odometry to the specified pose
     *
     * @param pose the pose to reset the odometry to
     */
    fun resetOdometry(pose: Pose2d?) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.yaw),
            modules.map { it.location.toSwerveModulePosition() }.toTypedArray(),
            pose
        )
    }

    /**
     * drive the robot using the specified chassis speeds
     *
     * @param chassisSpeeds the chassis speeds to drive at
     * @param fieldRelative whether the chassis speeds are field-relative
     */
    fun drive(chassisSpeeds: ChassisSpeeds, fieldRelative: Boolean) {
        val chassisSpeedsField =
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d.fromDegrees(gyro.yaw))
            else chassisSpeeds
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            chassisSpeedsField
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            chassisSpeedsField,
            4.0,
            2.0,
            2 * PI
        )

//        SwerveDriveKinematics.desaturateWheelSpeeds()

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

        modules.forEachIndexed { i, module ->
            module.setpoint = SwerveModuleState(
                desiredStates[i]?.speedMetersPerSecond ?: 0.0,
                desiredStates[i]?.angle ?: Rotation2d()
            )
        }
    }

    /**
     * Zeroes the heading of the robot
     */
    fun zeroHeading() {
        gyro.yaw = 0.0
    }

    /**
     * the heading of the robot.
     */
    val heading: Double
        get() = poseEstimator.estimatedPosition.rotation.degrees

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    val turnRate: Double
        get() = gyro.yaw
}

private fun Translation2d.toSwerveModulePosition(): SwerveModulePosition = SwerveModulePosition(this.norm, this.angle)

