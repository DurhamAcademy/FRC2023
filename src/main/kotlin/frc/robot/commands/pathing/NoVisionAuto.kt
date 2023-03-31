package frc.robot.commands.pathing

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.drivetrain.DriveCommand
import frc.robot.commands.elevator.ZeroElevatorAndIdle
import frc.robot.commands.intake.IdleIntake
import frc.robot.commands.manipulator.Throw
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementLevel

class NoVisionAuto(
    private var robotContainer: RobotContainer,
) : Auto {

    override fun getCommand(): Command {
        return SetSubsystemPosition(
            robotContainer,
            { IOLevel.High },
            { GamePiece.cone },
            true
        )
            .andThen(
                Throw(
                    robotContainer.manipulator,
                    { GamePiece.cone },
                    { PlacementLevel.Level3 }
                )
                    .withTimeout(1.5))
            .andThen(
                DriveCommand(
                    robotContainer.drivetrain,
                    { 0.8 },
                    { 0.0 },
                    { 0.0 },
                    false,
                    { Translation2d(0.0, 0.0) })
            )
            .withTimeout(8.0)
    }
}