package frc.robot.commands.pathing

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.balance.AutoBalance
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.building.blocks.BuildingBlocks
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide

class AutoPlaceAndBalance(
    private val robotContainer: RobotContainer,
) : Auto {
    override fun getCommand(): Command {
        return ConditionalCommand(
            pathRed(robotContainer),
            ConditionalCommand(
                pathBlue(robotContainer),
                PrintCommand("No alliance")
            ) { Game.alliance == DriverStation.Alliance.Blue }
        ) { Game.alliance == DriverStation.Alliance.Red }
    }

    private fun pathRed(robotContainer: RobotContainer): Command =
        BuildingBlocks.goToPlacementPoint(
            robotContainer.drivetrain,
            robotContainer.arm,
            IOLevel.High,
            PlacementGroup.Middle,
            PlacementSide.CloseCone
        )
            .deadlineWith(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.High },
                    { GamePiece.cone },
                    true
                )
            )
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.High },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to place cone on L3 middle
            .andThen(
                Throw(
                    robotContainer.manipulator,
                    { GamePiece.cone },
                    { PlacementLevel.Level3 }).withTimeout(0.75)
            ) // shoot cube
            .withTimeout(4.0)
            .andThen(
                MoveToPosition(robotContainer.drivetrain, 14.4, 2.72, 180.0, maxPosSpeed = 3.7)
                    .andThen(
                        MoveToPosition(
                            robotContainer.drivetrain,
                            (13.34 - 0.5),
                            2.72,
                            180.0,
                            maxPosSpeed = 3.7
                        ).withTimeout(4.0)
                    )
                    .alongWith(
                        SetSubsystemPosition(robotContainer, { IOLevel.Balance }, { GamePiece.cone }, true)
                    )
            )
            .andThen(AutoBalance(robotContainer.drivetrain))

    private fun pathBlue(robotContainer: RobotContainer): Command {
        return BuildingBlocks.goToPlacementPoint(robotContainer.drivetrain, robotContainer.arm, IOLevel.High, PlacementGroup.Middle, PlacementSide.CloseCone)
            .deadlineWith(SetSubsystemPosition(
                robotContainer,
                { IOLevel.High },
                { GamePiece.cone },
                true
            ))
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.High },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to place cone on L3 middle
            .andThen(
                Throw(
                    robotContainer.manipulator,
                    { GamePiece.cone },
                    { PlacementLevel.Level3 }).withTimeout(0.5)
            ) // shoot cube
            .withTimeout(4.0)
            .andThen(
                MoveToPosition(robotContainer.drivetrain,  1.95/*16.52 - 14.4*/, 2.72, 0.0,
                    maxPosSpeed = 3.7
                )
                    .andThen(
                        MoveToPosition(robotContainer.drivetrain, 16.52 - (13.34 - 0.5), 2.72, 0.0,
                            maxPosSpeed = 3.7
                        )
                            .withTimeout(4.0)
                    )
                    .alongWith(
                        SetSubsystemPosition(robotContainer, { IOLevel.Balance }, { GamePiece.cone }, true)
                    )
            )
            .andThen(AutoBalance(robotContainer.drivetrain))
    }
}