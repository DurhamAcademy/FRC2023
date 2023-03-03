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

    // these entries are used to debug how fast the robot wants to move to get
    // to the desired position
//    val speedx = drivetrain.Idrc.add("speedx1$x.$y.$angle", 0.0)
//        .entry
//    val speedy = drivetrain.Idrc.add("speedy1$x.$y.$angle", 0.0)
//        .entry
//    val speedr = drivetrain.Idrc.add("speedr1$x.$y.$angle", 0.0)
//        .entry

    override fun execute() {
        // get the current position of the robot and offset it by the desired position
        val current = drivetrain.poseEstimator.estimatedPosition
        val desired = Pose2d(x, y, Rotation2d(angle))
        var offset = current.minus(desired)

        // if the robot is within 0.1 radians of the desired angle, set the
        // rotation movement to 0 so that there isnt any overshoot or oscillation
        if (offset.rotation.radians.absoluteValue < 0.1) {
            offset = Transform2d(offset.translation, Rotation2d())
        }
        // same as above but for movement in the x and y directions
        if (offset.translation.norm.absoluteValue < 0.1) {
            offset = Transform2d(Translation2d(), offset.rotation)
        }

        // convert the offset to chassis speeds and clamp the values to be between
        // reasonable values
        val speeds = ChassisSpeeds(
            (offset.translation.x * 10.0).coerceIn(-0.5, 0.5),
            (offset.translation.y * 10.0).coerceIn(-0.5, 0.5),
            (offset.rotation.radians * 1.0).coerceIn(-1.0, 1.0)
        )

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
        return drivetrain.poseEstimator.estimatedPosition.minus(Pose2d(x, y, Rotation2d(angle))).translation.norm < 0.1
    }

    override fun end(interrupted: Boolean) {
        drivetrain.drive(ChassisSpeeds(), true)
    }
}