package frc.robot.commands.pathing

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.max
import frc.robot.constants.drivetrain as drivetrainConstants

val Pose2d.flipped: Pose2d
    get() = Pose2d(
        Translation2d(
            -(this.translation.x - 8.3) + 8.3,
            this.translation.y
        ),
        -rotation
    )
//var lastPrintTime: Double = Timer.getFPGATimestamp()
//var queuedPrints: String = ""
//
//fun safePrint(vararg message: String, separator: String = "") {
//    message.joinToString {  }
//}
open class MoveToPosition(
    private val drivetrain: Drivetrain,
    /**
     * The desired position of the robot (in meters)
     */
    private inline var pose: (
        xPID: ProfiledPIDController,
        yPID: ProfiledPIDController,
        rPID: ProfiledPIDController
    ) -> Pose2d,
    /**
     * The desired velocity of the robot (in meters per second)
     */
    private val velocity: Transform2d = Transform2d(),
    private val toleranceppos: Double = 0.02,
    private val tolerancepvel: Double = 0.1,
    private val tolerancerpos: Double = 0.01,
    private val tolerancervel: Double = 0.1,
    private val snapMode: Boolean = false,
    private val maxPosSpeed: Double = 3.0,
    private val maxRotSpeed: Double = PI / 2.0,
) : CommandBase() {
    constructor(drivetrain: Drivetrain, x: Double = 0.0, y: Double = 0.0, angle: Double = 0.0, maxPosSpeed: Double = 3.0,
                maxRotSpeed: Double = PI / 2.0,) : this(
        drivetrain,
        Pose2d(x, y, Rotation2d.fromDegrees(angle)),
        Transform2d(
            Translation2d(0.0, 0.0),
            Rotation2d.fromDegrees(0.0)
        ),
        maxPosSpeed,
        maxRotSpeed
    )

    constructor(
        drivetrain: Drivetrain,
        pose: Pose2d,
        velocity: Transform2d = Transform2d(),
        toleranceppos: Double = 0.075,
        tolerancepvel: Double = 0.1,
        tolerancerpos: Double = 0.01,
        tolerancervel: Double = 0.1,
        snapMode: Boolean = false
    ) : this(
        drivetrain,
        { _, _, _ -> pose },
        velocity,
        toleranceppos,
        tolerancepvel,
        tolerancerpos,
        tolerancervel,
        snapMode
    )

    init {
        addRequirements(drivetrain)
    }

    // these entries are used to debug how fast the robot wants to move to get
    // to the desired position
//    val speedx = drivetrain.Idrc.add("speedx1${Random.nextDouble()}", 0.0)
//        .entry
//    val speedy = drivetrain.Idrc.add("speedy1${Random.nextDouble()}", 0.0)
//        .entry
//    val speedr = drivetrain.Idrc.add("speedr1${Random.nextDouble()}", 0.0)
//        .entry

    private val xPIDController = ProfiledPIDController(
        Companion.xP, 0.0, 0.05, TrapezoidProfile.Constraints(
            maxPosSpeed,
            max(10.0, drivetrainConstants.maxAcceleration)
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.x, 0.0)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    private val yPIDController = ProfiledPIDController(
        Companion.yP, 0.0, 0.05, TrapezoidProfile.Constraints(
            maxPosSpeed,
            max(10.0, drivetrainConstants.maxAcceleration)
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.y, 0.0)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    private val rPIDController = ProfiledPIDController(
        Companion.rP, 0.0, 0.0, TrapezoidProfile.Constraints(
            maxRotSpeed, max(PI, drivetrainConstants.maxAngularAcceleration)
        )
    ).also {
        it.enableContinuousInput(-PI, PI)
        it.reset(
            drivetrain.estimatedPose2d.rotation.radians,
            0.0
        )
        it.setTolerance(tolerancerpos, tolerancervel)
    }

    val start = drivetrain.estimatedPose2d

    val visualization = drivetrain.field2d.getObject("MoveToPosition")
    override fun initialize() {
        if (snapMode) pose = { _, _, _ ->
            SnapToPostion.closestPose(drivetrain)
        }
        xPIDController.reset(drivetrain.estimatedPose2d.translation.x, drivetrain.estimatedVelocity.translation.x)
        yPIDController.reset(drivetrain.estimatedPose2d.translation.y, drivetrain.estimatedVelocity.translation.y)
        rPIDController.reset(drivetrain.estimatedPose2d.rotation.radians, drivetrain.estimatedVelocity.rotation.radians)

        visualization.pose = pose(xPIDController, yPIDController, rPIDController)
    }

    // on command start and every time the command is executed, calculate the

    override fun execute() {
        if (!drivetrain.canTrustPose) return initialize()

        val current = drivetrain.estimatedPose2d
        val desired = pose(xPIDController, yPIDController, rPIDController)

        // BAIL IF WANT TO MOVE REALLY FAR
        if (desired.translation.getDistance(current.translation) > 8.0) {
            drivetrain.drive(ChassisSpeeds(), true)
            return
        }

        visualization.pose = desired

        // log the current position and the desired position
        SmartDashboard.putNumber("curr-x", current.translation.x)
        SmartDashboard.putNumber("curr-y", current.translation.y)
        SmartDashboard.putNumber("curr-r", current.rotation.radians)
        SmartDashboard.putNumber("des-x", desired.translation.x)
        SmartDashboard.putNumber("des-y", desired.translation.y)
        SmartDashboard.putNumber("des-r", desired.rotation.radians)


        // calculate the speeds needed to get to the desired position
        val speeds = ChassisSpeeds(
            xPIDController.calculate(
                current.translation.x,
                TrapezoidProfile.State(
                    desired.translation.x,
                    velocity.translation.x
                )
            ),
            yPIDController.calculate(
                current.translation.y,
                TrapezoidProfile.State(
                    desired.translation.y,
                    velocity.translation.y
                )
            ),
            rPIDController.calculate(
                current.rotation.radians,
                TrapezoidProfile.State(
                    desired.rotation.radians,
                    velocity.rotation.radians
                )
            )
        )

        SmartDashboard.putNumber("xpiderr", xPIDController.positionError)
        SmartDashboard.putNumber("ypiderr", yPIDController.positionError)
        SmartDashboard.putNumber("rpiderr", rPIDController.positionError)

        SmartDashboard.putNumber("xpidvel", xPIDController.velocityError)
        SmartDashboard.putNumber("ypidvel", yPIDController.velocityError)
        SmartDashboard.putNumber("rpidvel", rPIDController.velocityError)

        SmartDashboard.putData("xpid", xPIDController)
        SmartDashboard.putData("ypid", yPIDController)
        SmartDashboard.putData("rpid", rPIDController)

        SmartDashboard.putNumber("xpidPosTolerance", xPIDController.positionTolerance)
        SmartDashboard.putNumber("ypidPosTolerance", yPIDController.positionTolerance)
        SmartDashboard.putNumber("rpidPosTolerance", rPIDController.positionTolerance)

        SmartDashboard.putNumber("xpidVelTolerance", xPIDController.velocityTolerance)
        SmartDashboard.putNumber("ypidVelTolerance", yPIDController.velocityTolerance)
        SmartDashboard.putNumber("rpidVelTolerance", rPIDController.velocityTolerance)

        // at goal
        SmartDashboard.putBoolean("xpidAtGoal", xPIDController.atGoal())
        SmartDashboard.putBoolean("ypidAtGoal", yPIDController.atGoal())
        SmartDashboard.putBoolean("rpidAtGoal", rPIDController.atGoal())

        SmartDashboard.putNumber("SPEED", hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
//        val speeds = ChassisSpeeds(
//          1.0, 0.0, 0.0
//        )

        // set the debug entries to the speeds so we can see values in the
        // smartdashboard
//        speedx.setDouble(speeds.vxMetersPerSecond)
//        speedy.setDouble(speeds.vyMetersPerSecond)
//        speedr.setDouble(speeds.omegaRadiansPerSecond)


        // tell the drivetrain to drive at the calculated speeds
        drivetrain.drive(speeds, true)
    }

    override fun isFinished(): Boolean {
        // stop when the robot is within 0.1 meters of the desired position
        return xPIDController.atGoal() && yPIDController.atGoal() && rPIDController.atGoal()
        //return drivetrain.estimatedPose2d.minus(Pose2d(pose().translation, Rotation2d())).translation.norm < toleranceppos
        //       && rPIDController.atGoal()
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(0.0, 0.0, 0.0), true)
    }

    val flipped: MoveToPosition
        get() = MoveToPosition(
            drivetrain,
            { x, y, r -> pose(x, y, r).flipped },
            velocity,
            toleranceppos,
            tolerancepvel,
            tolerancerpos,
            tolerancervel,
            snapMode
        )

    companion object {
        const val rP = 8.0
        const val yP = 5.0
        const val xP = 5.0

        fun snapToYValue(
            drivetrain: Drivetrain,
            y: () -> Double,
            yTolerance: Double = 0.1,
            r: () -> Rotation2d = { drivetrain.estimatedPose2d.rotation },
            rTolerance: Double = 0.1,
        ) =
            run {
                (drivetrain.estimatedPose2d)
                MoveToPosition(
                    drivetrain,
                    { _, _, _ ->
                        Pose2d(
                            drivetrain.estimatedPose2d.translation.x,
                            y(),
                            r()
                        )
                    },
                    toleranceppos = yTolerance,
                    tolerancerpos = rTolerance
                ).withTimeout(3.5)
            }

        /**
         * @param rotValues The angle in radians
         */
        fun snapToScoring(
            drivetrain: Drivetrain,
            yValues: () -> Iterable<Double>,
            rotValues: () -> Iterable<Double>
        ): Command =
            snapToYValue(
                drivetrain,
                {
                    yValues().minByOrNull { value ->
                        (value - drivetrain.estimatedPose2d.y).absoluteValue
                    } ?: drivetrain.estimatedPose2d.y
                },
                yTolerance = 0.05,
                {
                    Rotation2d.fromRadians(
                        rotValues().minByOrNull { value ->
                            (
                                    MathUtil.angleModulus(value)
                                            -
                                            MathUtil.angleModulus(
                                                drivetrain.estimatedPose2d.rotation.radians
                                            )
                                    ).absoluteValue
                        } ?: drivetrain.estimatedPose2d.rotation.radians
                    )
                },
                rTolerance = 0.15
            )

    }
}

//this function should be used when copying and pasting an auto function, and you need to flip the x-coordinates.
fun flipped(x: Double): Double {
    val newX: Double
    if (x < 8.3)
        newX = 8.3 + (8.3 - x)
    else
        newX = 8.3 - (x - 8.3)
    return newX
}
