package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
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
    private val x: Double,
    private val y: Double,
    private val angle: Double,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    // these entries are used to debug how fast the robot wants to move to get
    // to the desired position
    val speedx = drivetrain.Idrc.add("speedx1$x.$y.$angle${Random.nextDouble()}", 0.0)
        .entry
    val speedy = drivetrain.Idrc.add("speedy1$x.$y.$angle${Random.nextDouble()}", 0.0)
        .entry
    val speedr = drivetrain.Idrc.add("speedr1$x.$y.$angle${Random.nextDouble()}", 0.0)
        .entry


    val xPIDController = ProfiledPIDController(
        1.0, 0.0, 0.0, TrapezoidProfile.Constraints(
            2.0,
            0.1
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.x, 0.0)
    }
    val yPIDController = ProfiledPIDController(
        1.0, 0.0, 0.0, TrapezoidProfile.Constraints(
            2.5,
            0.1
        )
    ).also {
        it.reset(drivetrain.estimatedPose2d.translation.y, 0.0)
    }
    val rPIDController = ProfiledPIDController(
        1.0, 0.0, 0.0, TrapezoidProfile.Constraints(
            PI / 2,
            0.1
        )
    ).also {
        it.enableContinuousInput(-PI, PI)
        it.reset(drivetrain.estimatedPose2d.rotation.radians, 0.0)
    }

    override fun initialize() {
        xPIDController.reset(drivetrain.estimatedPose2d.translation.x, 0.0)
        yPIDController.reset(drivetrain.estimatedPose2d.translation.y, 0.0)
        rPIDController.reset(drivetrain.estimatedPose2d.rotation.radians, 0.0)
    }

    // on command start and every time the command is executed, calculate the

    override fun execute() {
        val current = drivetrain.estimatedPose2d
        val desired = Pose2d(x, y, Rotation2d.fromDegrees(angle))

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
                desired.translation.x,
            ),
            yPIDController.calculate(
                current.translation.y,
                desired.translation.y,
            ),
            rPIDController.calculate(
                current.rotation.radians,
                desired.rotation.radians,
            )
        )
        SmartDashboard.putNumber("xpiderr", xPIDController.positionError)
        SmartDashboard.putNumber("ypiderr", yPIDController.positionError)
        SmartDashboard.putNumber("rpiderr", rPIDController.positionError)

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
        return drivetrain.estimatedPose2d.minus(Pose2d(x, y, Rotation2d(angle))).translation.norm < 0.05
                && drivetrain.estimatedPose2d.rotation.minus(Rotation2d(angle)).radians < 0.05
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(), true)
        // reset the PID controllers
        xPIDController.reset(0.0, 0.0)
        yPIDController.reset(0.0, 0.0)
        rPIDController.reset(0.0, 0.0)
    }
}