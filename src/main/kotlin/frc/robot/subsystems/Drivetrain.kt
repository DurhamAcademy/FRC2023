package frc.robot.subsystems

import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.robot.PhotonCameraWrapper
import frc.robot.RobotContainer
import frc.robot.commands.drivetrain.DriverCommand
import frc.robot.constants.Constants
import frc.robot.constants.drivetrain
import frc.robot.controls.ControlScheme
import kotlin.math.PI

class Drivetrain(
    controlScheme: ControlScheme,
    val cameraWrappers: List<PhotonCameraWrapper>,
    val robotContainer: RobotContainer
) : SubsystemBase() {

    init {
        defaultCommand = DriverCommand(this, controlScheme, { robotContainer.rotateTo180 })
    }

    private val swerveTab = getTab("Swerve Diagnostics")

    //private PowerDistribution PDP = new PowerDistribution();
    private val xSpeedEntry = swerveTab.add("xBox xSpeed", 0)
        .entry
    private val ySpeedEntry = swerveTab.add("xBox ySpeed", 0)
        .entry
    private val rotEntry = swerveTab.add("xBox rot", 0)
        .entry
    val invertx = swerveTab.add("invert x", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .entry
    val inverty = swerveTab.add("invert y", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .entry
    val invertrot = swerveTab.add("invert rot", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .entry
    private val frontLeft = SwerveModule( // front right
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
    private val frontRight = SwerveModule( // backleft
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
    private val backLeft = SwerveModule(
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
    private val backRight = SwerveModule(
        Constants.BRDriveMotorId,
        Constants.BRTurnMotorId,
        Constants.BRTurnEncoderId,
        "backRight",

        Translation2d(
            -Constants.MODULE_DISTANCE_X / 2,
            -Constants.MODULE_DISTANCE_Y / 2
        ),
        angleZero = Constants.BRZeroAngle,
    )
    val modules = listOf(frontLeft, frontRight, backLeft, backRight)
    private val kinematics = SwerveDriveKinematics(
        *modules.map { it.location }.toTypedArray()
    )

    internal val gyro = Pigeon2(20, "rio").apply {
        configFactoryDefault()
    }

    private val f2d = Field2d()

    val Idrc = getTab("drivetrain")

    // pose shuffleboard stuff (using the field 2d widget)
    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.fromDegrees(gyro.yaw),
        modules.map {
            it.swerveModulePosition
        }.toTypedArray(),
        Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(1.8, 1.8, 1.8)
    )

    @Deprecated("Use estimatedPose2d instead, this is only for internal drivetrain use")
    var simEstimatedPose2d: Pose2d = Pose2d()

    inline val estimatedPose2d: Pose2d
        get() = if (!Game.sim) {
            poseEstimator.estimatedPosition
        } else {
            simEstimatedPose2d
        }

    private var poseList = MutableList(10) { Pose2d() }
    private var timeList = MutableList(10) { it.toDouble() }
    val estimatedVelocity: Transform2d
        get() = if (Game.sim) {
            // get average difference between last 10 poses and divide by
            // average difference between last 10 timestamps
            val averagePoseXDiff = poseList.mapIndexedNotNull { index, pose2d ->
                if (index == 0) null
                else pose2d.translation.x - poseList[index - 1].translation.x
            }.average()
            val averagePoseYDiff = poseList.mapIndexedNotNull { index, pose2d ->
                if (index == 0) null
                else pose2d.translation.y - poseList[index - 1].translation.y
            }.average()
            val averagePoseThetaDiff = poseList.mapIndexedNotNull { index, pose2d ->
                if (index == 0) null
                else pose2d.rotation.radians - poseList[index - 1].rotation.radians
            }.average()
            val averageTimeDiff = timeList.mapIndexedNotNull { index, time ->
                if (index == 0) null
                else time - timeList[index - 1]
            }.average()
            Transform2d(
                Translation2d(averagePoseXDiff / averageTimeDiff, averagePoseYDiff / averageTimeDiff),
                Rotation2d(averagePoseThetaDiff / averageTimeDiff)
            )
        } else {
            Transform2d()
        }

    val field2d = Field2d().apply {
        this.robotPose = estimatedPose2d
    }

    private val gyroPitchWidget = Idrc.add("Gyro Pitch", 0.0)
        .withWidget("Gryo")
        .withProperties(mapOf("min" to -180.0, "max" to 180.0))
        .withSize(2, 2)
        .entry
    private val gyroYawWidget = Idrc.add("Gyro Yaw", 0.0)
        .withWidget("Gryo")
        .withProperties(mapOf("min" to -180.0, "max" to 180.0))
        .withSize(2, 2)
        .entry
    private val gyroRollWidget = Idrc.add("Gyro Roll", 0.0)
        .withWidget("Gryo")
        .withProperties(mapOf("min" to -180.0, "max" to 180.0))
        .withSize(2, 2)
        .entry

    private var i = 0U

    /**
     * The periodic method is run roughly every 20ms. This is where we update
     * any values that are constantly changing, such as the robot's position,
     * the robot's heading, and the robot's speed
     *
     * @author A1cD
     */
    override fun periodic() {
        // This method will be called once per scheduler run

        // update the gyro
        gyroPitchWidget.setDouble(gyro.pitch)
        gyroYawWidget.setDouble(gyro.yaw)
        gyroRollWidget.setDouble(gyro.roll)

        // pose estimator handles odometry too
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
                // and make sure we aren't adding a measurement that is too new
                // add the vision measurement to the drivetrain pose estimator
                poseEstimator.addVisionMeasurement(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    estimatedRobotPose.timestampSeconds
                )
            }
        }

        SmartDashboard.putData("huhhh", this)

        SmartDashboard.putNumber("gyroangle", gyro.yaw)
        SmartDashboard.putNumber("uptime", gyro.upTime.toDouble())
        SmartDashboard.putNumber("posex", poseEstimator.estimatedPosition.translation.x)
        SmartDashboard.putNumber("posey", poseEstimator.estimatedPosition.translation.y)

        if (Game.sim) {
            val vel = estimatedVelocity
            simEstimatedPose2d = simEstimatedPose2d + (vel * 0.02 * 0.01) + simQueuedForce
            simQueuedForce = Transform2d()
        }
        // update the field 2d widget with the current robot position
        if (i++ >= 0U) {
            field2d.robotPose = estimatedPose2d
            i = 0U
        }
        // push to shuffleboard
        SmartDashboard.putData("field", field2d)

        // update the last pose
        poseList.removeAt(0)
        poseList.add(estimatedPose2d)
        timeList.removeAt(0)
        timeList.add(Timer.getFPGATimestamp())
    }

    private var simQueuedForce = Transform2d()


    val xSlewRateLimiter = SlewRateLimiter(drivetrain.maxAcceleration)
    val ySlewRateLimiter = SlewRateLimiter(drivetrain.maxAcceleration)
    val rotSlewRateLimiter = SlewRateLimiter(drivetrain.maxAngularAcceleration)

    /**
     * drive the robot using the specified chassis speeds
     *
     * @param chassisSpeeds the chassis speeds to drive at
     * @param fieldRelative whether the chassis speeds are field-relative
     */
    fun drive(
        chassisSpeeds: ChassisSpeeds,
        fieldRelative: Boolean,
        rotAxis: Translation2d = Translation2d(0.0, 0.0)
    ) {
        val chassisSpeedsField =
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,//.slewLimited(xSlewRateLimiter, ySlewRateLimiter, rotSlewRateLimiter),
                estimatedPose2d.rotation
            )
            else chassisSpeeds
        val swerveModuleStates = kinematics.toSwerveModuleStates(
            chassisSpeedsField,
            rotAxis
        )

        val currentChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates)

        SwerveDriveKinematics.desaturateWheelSpeeds(
            /* moduleStates = */ swerveModuleStates,
            /* currentChassisSpeed = */ currentChassisSpeeds,
            /* attainableMaxModuleSpeedMetersPerSecond = */ 4.0,
            /* attainableMaxTranslationalSpeedMetersPerSecond = */ 4.0,
            /* attainableMaxRotationalVelocityRadiansPerSecond = */ PI,
        )

        if (Game.real) {
            swerveModuleStates.forEachIndexed { i, swerveModuleState ->
                modules[i].setpoint = swerveModuleState
            }
            // Telemetry
            xSpeedEntry.setDouble(chassisSpeeds.vxMetersPerSecond)
            ySpeedEntry.setDouble(chassisSpeeds.vyMetersPerSecond)
            rotEntry.setDouble(chassisSpeeds.omegaRadiansPerSecond)
        } else {
            val newChassisSpeeds = kinematics.toChassisSpeeds(*swerveModuleStates)
            // add the chassis speeds to the sim pose with dt = 0.02
            // also retain velocity when told to stop

//            simEstimatedPose2d = Pose2d(
//                simEstimatedPose2d.translation.x + newChassisSpeeds.vxMetersPerSecond * 0.02,
//                simEstimatedPose2d.translation.y + newChassisSpeeds.vyMetersPerSecond * 0.02,
//                simEstimatedPose2d.rotation.plus(Rotation2d(newChassisSpeeds.omegaRadiansPerSecond) * 0.02)
//            ).apply {
//                if (chassisSpeeds.vxMetersPerSecond == 0.0 && chassisSpeeds.vyMetersPerSecond == 0.0) {
//                    simEstimatedPose2d = Pose2d(simEstimatedPose2d.translation, simEstimatedPose2d.rotation)
//                    simEstimatedPose2d.plus(estimatedVelocity * 0.02)
//                } else {
//                    simEstimatedPose2d = Pose2d(simEstimatedPose2d.translation, simEstimatedPose2d.rotation)
//                    simEstimatedPose2d.plus(estimatedVelocity * 0.02 * 0.1)
//                }
//            }
            simQueuedForce = Transform2d(
                Translation2d(
                    newChassisSpeeds.vxMetersPerSecond * 0.02,
                    newChassisSpeeds.vyMetersPerSecond * 0.02
                ),
                Rotation2d(newChassisSpeeds.omegaRadiansPerSecond * 0.02)
            )
        }
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

    inline val canTrustPose: Boolean
        get() = cameraWrappers.any { it.canTrustPose } || Game.sim

}

fun ChassisSpeeds.slewLimited(
    xSlewRateLimiter: SlewRateLimiter,
    ySlewRateLimiter: SlewRateLimiter,
    rotSlewRateLimiter: SlewRateLimiter,
) = ChassisSpeeds(
    xSlewRateLimiter.calculate(this.vxMetersPerSecond),
    ySlewRateLimiter.calculate(this.vyMetersPerSecond),
    rotSlewRateLimiter.calculate(this.omegaRadiansPerSecond)
)