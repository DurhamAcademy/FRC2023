package frc.robot.commands.pathing

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.manipulator.Throw
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementLevel

class OnlyPlaceConeAuto(
    private var robotContainer: RobotContainer,
) : Auto {

    override fun getCommand(): Command {
        return ConditionalCommand(
            pathRed(robotContainer),
            ConditionalCommand(
                pathBlue(robotContainer),
                PrintCommand("No alliance"),
                { Game.alliance == DriverStation.Alliance.Blue }
            ),
            { Game.alliance == DriverStation.Alliance.Red }
        )
    }


    fun pathRed(robotContainer: RobotContainer): Command {
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
    }


    fun pathBlue(robotContainer: RobotContainer): Command {
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
    }
}