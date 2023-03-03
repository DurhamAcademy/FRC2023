package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import frc.kyberlib.command.Game
import frc.robot.Constants
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain

class DriveCommand(
    var drivetrain: Drivetrain,
    var controlScheme: ControlScheme,
) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }
    override fun execute() {
        val alianceMulitplier = when (Game.alliance) {
            Alliance.Invalid -> 1.0
            Alliance.Blue -> Constants.Field2dLayout.Axes.Blue.fieldOffsetMultiplier
            Alliance.Red -> Constants.Field2dLayout.Axes.Red.fieldOffsetMultiplier
        }
        val vec = Translation2d(-controlScheme.forward, controlScheme.strafe)
            .times(2.0)
        drivetrain.drive(
            ChassisSpeeds(
                vec.x * Constants.powerPercent * alianceMulitplier,
                vec.y * Constants.powerPercent,
                -controlScheme.rotation *2 * Math.PI *
                        Constants.powerPercent *.25
            ),
            true,
            Translation2d() // chris wants in the middle
        )
    }
}