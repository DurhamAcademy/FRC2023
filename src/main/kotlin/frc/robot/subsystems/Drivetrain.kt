package frc.robot.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.PhotonCameraWrapper
import frc.robot.commands.DriveCommand
import frc.robot.controls.ControlScheme
import java.lang.Math.PI

open class Drivetrain(
    controlScheme: ControlScheme,
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
    val frontLeft = SwerveModule( // front right
        Constants.FLDriveMotorId,
        Constants.FLTurnMotorId,
        Constants.FLTurnEncoderId,
        "frontLeft",
        angleZero = Constants.FLZeroAngle,
        location = Translation2d(
            Constants.MODULE_DISTANCE_X / 2,
            Constants.MODULE_DISTANCE_Y / 2
        )
    )
    val frontRight = SwerveModule( // backleft
        Constants.FRDriveMotorId,
        Constants.FRTurnMotorId,
        Constants.FRTurnEncoderId,
        "frontRight",
        angleZero = Constants.FRZeroAngle,
        location = Translation2d(
            Constants.MODULE_DISTANCE_X / 2,
            -Constants.MODULE_DISTANCE_Y / 2
        )
    )
    val backLeft = SwerveModule(
        Constants.BLDriveMotorId,
        Constants.BLTurnMotorId,
        Constants.BLTurnEncoderId,
        "backLeft",
        Translation2d(
            -Constants.MODULE_DISTANCE_X / 2,
            Constants.MODULE_DISTANCE_Y / 2
        ),
        angleZero = Constants.BLZeroAngle,
    )
    val backRight = SwerveModule(
        Constants.BRDriveMotorId,
        Constants.BRTurnMotorId,
        Constants.BRTurnEncoderId,
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
    val Idrc = getTab("drivetrain")

    // pose shuffleboard stuff (using the field 2d widget)
    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.fromDegrees(gyro.yaw),
        modules.map {
            it.swerveModulePosition
        }.toTypedArray(),
        Pose2d()
    )

    private var simEstimatedPose2d: Pose2d = Pose2d()

    open val estimatedPose2d: Pose2d
        get() = if (RobotBase.isReal()) {
            poseEstimator.estimatedPosition
        } else {
            simEstimatedPose2d
        }

    val field2d = Field2d().apply {
        this.robotPose = estimatedPose2d
    }

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
        // we want to get the estimated position from each camera's pose
        // estimator and add it to the drivetrain's pose estimator. This helps
        // us get a more accurate position estimate
        cameraWrappers.forEach { cameraWrapper ->
            val pose = cameraWrapper.getEstimatedGlobalPose(poseEstimator.estimatedPosition)
            pose.ifPresent { estimatedRobotPose -> // if the camera sees a target,
                /*
                 * we want to add the vision measurement to the drivetrain pose
                 */
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
        cameraWrappers[0].getEstimatedGlobalPose(poseEstimator.estimatedPosition).ifPresent {
            SmartDashboard.putNumber("poseyCamera", it.estimatedPose.translation.y)
            SmartDashboard.putNumber("posexCamera", it.estimatedPose.translation.x)
        }
//        gyroEntry.setDouble(gyro.yaw)
//        odometry.update(
//            Rotation2d.fromDegrees(gyro.yaw),
//            modules.map { it.position.toSwerveModulePosition() }.toTypedArray()
//        )

        // update the field 2d widget with the current robot position
        field2d.robotPose = estimatedPose2d
        // push to shuffleboard
        SmartDashboard.putData(field2d)
    }


    /**
     * drive the robot using the specified chassis speeds
     *
     * @param chassisSpeeds the chassis speeds to drive at
     * @param fieldRelative whether the chassis speeds are field-relative
     */
    open fun drive(chassisSpeeds: ChassisSpeeds, fieldRelative: Boolean) {
        val chassisSpeedsField =
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                poseEstimator.estimatedPosition.rotation
            )
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

        if (RobotBase.isReal()) {
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
        } else {
            val newChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates)
            // add the chassis speeds to the sim pose with dt = 0.02
            simEstimatedPose2d = Pose2d(
                simEstimatedPose2d.translation.x + newChassisSpeeds.vxMetersPerSecond * 0.02,
                simEstimatedPose2d.translation.y + newChassisSpeeds.vyMetersPerSecond * 0.02,
                simEstimatedPose2d.rotation.plus(Rotation2d(newChassisSpeeds.omegaRadiansPerSecond) * 0.02)
            )
        }
    }

    /**
     * Zeroes the heading of the robot
     */
    fun zeroHeading() {
        gyro.yaw = 0.0
    }

}

