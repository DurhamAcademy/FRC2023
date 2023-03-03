package frc.robot.commands.pathing

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.random.Random

class MoveToPosition(
    private val drivetrain: Drivetrain,
    /**
     * The desired position of the robot (in meters)
     */
    private val pose: Pose2d,
    /**
     * The desired velocity of the robot (in meters per second)
     */
    private val velocity: Transform2d = Transform2d(),
    private val toleranceppos: Double = 0.1,
    private val tolerancepvel: Double = 0.01,
    private val tolerancerpos: Double = 0.025,
    private val tolerancervel: Double = 0.01,
) : CommandBase() {
    constructor(drivetrain: Drivetrain, x: Double = 0.0, y: Double = 0.0, angle: Double = 0.0) : this(
        drivetrain,
        Pose2d(x, y, Rotation2d.fromDegrees(angle)),
        Transform2d(Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
    )

    init {
        addRequirements(drivetrain)
    }

    // these entries are used to debug how fast the robot wants to move to get
    // to the desired position
    val speedx = drivetrain.Idrc.add("speedx1${Random.nextDouble()}", 0.0)
        .entry
    val speedy = drivetrain.Idrc.add("speedy1${Random.nextDouble()}", 0.0)
        .entry
    val speedr = drivetrain.Idrc.add("speedr1${Random.nextDouble()}", 0.0)
        .entry

    val xPIDController = ProfiledPIDController(
        Companion.xP, 0.0, 0.0, TrapezoidProfile.Constraints(
            4.0,
            3.0
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.x, drivetrain.estimatedVelocity.x)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    val yPIDController = ProfiledPIDController(
        Companion.yP, 0.0, 0.0, TrapezoidProfile.Constraints(
            4.0,
            3.0
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.y, drivetrain.estimatedVelocity.y)
        it.setTolerance(toleranceppos, tolerancepvel)
    }
    val rPIDController = ProfiledPIDController(
        Companion.rP, 0.0, 0.0, TrapezoidProfile.Constraints(
            PI / 2, PI / 1.5
        )
    ).also {
        it.enableContinuousInput(-PI, PI)
        it.reset(drivetrain.estimatedPose2d.rotation.radians, drivetrain.estimatedVelocity.rotation.radians)
        it.setTolerance(tolerancerpos, tolerancervel)
    }

    val start = drivetrain.estimatedPose2d

    override fun initialize() {
        xPIDController.reset(drivetrain.estimatedPose2d.translation.x, drivetrain.estimatedVelocity.x)
        yPIDController.reset(drivetrain.estimatedPose2d.translation.y, drivetrain.estimatedVelocity.y)
        rPIDController.reset(drivetrain.estimatedPose2d.rotation.radians, drivetrain.estimatedVelocity.rotation.radians)
    }

    // on command start and every time the command is executed, calculate the

    override fun execute() {
        val current = drivetrain.estimatedPose2d
        val desired = pose

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
        speedx.setDouble(speeds.vxMetersPerSecond)
        speedy.setDouble(speeds.vyMetersPerSecond)
        speedr.setDouble(speeds.omegaRadiansPerSecond)


        // tell the drivetrain to drive at the calculated speeds
        drivetrain.drive(speeds, true)
    }

    override fun isFinished(): Boolean {
        // stop when the robot is within 0.1 meters of the desired position
        return drivetrain.estimatedPose2d.minus(Pose2d(pose.translation, Rotation2d())).translation.norm < toleranceppos
                && drivetrain.estimatedPose2d.rotation.minus(Rotation2d(pose.rotation.radians)).radians < tolerancerpos
        return drivetrain.poseEstimator.estimatedPosition.minus(Pose2d(x, y, Rotation2d(angle))).translation.norm < 0.1
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(), true)
        // reset the PID controllers
        xPIDController.reset(0.0, 0.0)
        yPIDController.reset(0.0, 0.0)
        rPIDController.reset(0.0, 0.0)
    }

    companion object {
        const val rP = 3.0
        const val yP = 1.5
        const val xP = 1.5
    }
}