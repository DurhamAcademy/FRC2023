package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import kotlin.math.absoluteValue

class MoveToPosition(
    private val drivetrain: Drivetrain,
    private val x: Double,
    private val y: Double,
    private val angle: Double,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val speedx = drivetrain.Idrc.add("speedx1$x.$y.$angle", 0.0)
        .entry
    val speedy = drivetrain.Idrc.add("speedy1$x.$y.$angle", 0.0)
        .entry
    val speedr = drivetrain.Idrc.add("speedr1$x.$y.$angle", 0.0)
        .entry

    override fun execute() {
        // get the current position of the robot and offset it by the desired position
        val current = drivetrain.poseEstimator.estimatedPosition
        val desired = Pose2d(x, y, Rotation2d(angle))
        var offset = current.minus(desired)

        // calculate the desired speed of the robot
        // max speed is 0.2 meters per second
        // speed is proportional to the distance from the desired position
        // tries to move at the difference between the current location and the desired location
        // in 0.2 seconds
        if (offset.rotation.radians.absoluteValue < 0.1) {
            offset = Transform2d(offset.translation, Rotation2d())
        }
        if (offset.translation.norm.absoluteValue < 0.1) {
            offset = Transform2d(Translation2d(), offset.rotation)
        }

        val speeds = ChassisSpeeds(
            (offset.translation.x * 10.0).coerceIn(-0.5, 0.5),
            (offset.translation.y * 10.0).coerceIn(-1.0, 1.0),
            (offset.rotation.radians * 1.0).coerceIn(-1.0, 1.0)
        )

        speedx.setDouble(speeds.vxMetersPerSecond)
        speedy.setDouble(speeds.vyMetersPerSecond)
        speedr.setDouble(speeds.omegaRadiansPerSecond)


        // drive the robot
        drivetrain.drive(speeds, true)
    }

    override fun isFinished(): Boolean {
        // stop when the robot is within 0.1 meters of the desired position
        return drivetrain.poseEstimator.estimatedPosition.minus(Pose2d(x, y, Rotation2d(angle))).translation.norm < 0.1
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(), true)
    }
}