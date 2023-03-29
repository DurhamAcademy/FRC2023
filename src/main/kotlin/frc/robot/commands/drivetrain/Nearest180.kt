package frc.robot.commands.drivetrain

import edu.wpi.first.math.util.Units
import frc.robot.subsystems.Drivetrain
import kotlin.math.round

object Nearest180{
    fun findNearest180(
        drivetrain: Drivetrain
    ): Double {
        return Units.degreesToRadians(round((drivetrain.estimatedPose2d.rotation.degrees/180) * 180))
    }
}