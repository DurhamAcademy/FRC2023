package frc.robot.commands.pathing

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.Constants
import frc.robot.subsystems.Drivetrain

class SnapToPostion(
    val drivetrain: Drivetrain
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    fun closest(): Translation2d? =
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
    override fun initialize(){
        val closest = closest()
//        if (closest != null) {
//            if (closest.getDistance(
//                    drivetrain.poseEstimator.estimatedPosition.translation
//            ) < 1.0)
//        }//todo: yea
    }
}