package frc.robot.commands.pathing

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.kyberlib.command.Game
import frc.robot.Constants
import frc.robot.subsystems.Drivetrain

object SnapToPostion {
    fun closest(drivetrain: Drivetrain): Translation2d? =
        Constants.Field2dLayout.Axes.closest(
            drivetrain.poseEstimator.estimatedPosition.translation,
            (if (Game.alliance == DriverStation.Alliance.Blue)
                Constants.Field2dLayout.Axes.Blue
            else Constants.Field2dLayout.Axes.Red).run {
                if (drivetrain.poseEstimator.estimatedPosition.translation.y
                    > Constants.Field2dLayout.Axes.YInt.barrier
                ) this.loadingZonePlatforms
                else this.scoringPoints
            }
        )
    fun closestPose(drivetrain: Drivetrain): Pose2d =
        Pose2d(closest(drivetrain),drivetrain.poseEstimator.estimatedPosition.rotation)

}