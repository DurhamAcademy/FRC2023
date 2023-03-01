package frc.robot.commands

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
    var closest = Constants.Field2dLayout.Axes.closest(
        drivetrain.poseEstimator.estimatedPosition.translation,
        Constants.Field2dLayout.Axes.run {
            (if (Game.alliance == DriverStation.Alliance.Blue)
                Constants.Field2dLayout.Axes.Blue
            else Constants.Field2dLayout.Axes.Red).scoringPoints
        }
    )
    override fun initialize() = TODO("write go to position code")
}