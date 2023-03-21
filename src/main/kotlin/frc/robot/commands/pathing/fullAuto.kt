package frc.robot.commands.pathing

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.building.blocks.BuildingBlocks
import frc.robot.subsystems.*
import frc.robot.utils.GamePiece
import frc.robot.utils.Slider
import frc.robot.utils.grid.PlacementSide

fun fullAuto(
    drivetrain: Drivetrain,
    arm: Arm,
    elevator: Elevator,
    manipulator: Manipulator,
    selector: DashboardSelector,
    nextGamePieceCallback: (gamePiece: GamePiece) -> Unit,
//    command
): Command {
    return BuildingBlocks.goToPlacementPoint(
        drivetrain,
        arm,
        { selector.placementLevel.ioLevel },
        { selector.placementPosition },
        { selector.placementSide },
    ).deadlineWith(
        SetSubsystemPosition(elevator, arm, drivetrain, { IOLevel.Idle }, { GamePiece.cube }, false)
    ).andThen(
        SetSubsystemPosition(
            elevator, arm,
            drivetrain,
            { selector.placementLevel.ioLevel },
            { selector.placementSide.asObject },
            true
        )
    ).andThen(
        Throw(
            manipulator,
            { selector.placementSide.asObject },
            { selector.placementLevel },
        ).withTimeout(0.25)
            .andThen(
                ConditionalCommand(
                    selector.moveCommand(1, -1),
                    selector.moveCommand(1, 0)
                ) { selector.selected.first == 8 }
            )
    )
        .andThen(
            BuildingBlocks.leaveCommunityZone(drivetrain, arm)
                .deadlineWith(
                    SetSubsystemPosition(
                        elevator,
                        arm,
                        drivetrain,
                        { IOLevel.Idle },
                        { selector.placementSide.asObject },
                        true
                    )
                )
        ).andThen(
            BuildingBlocks.goToPickupZone(drivetrain, arm)
                .deadlineWith(
                    SetSubsystemPosition(
                        elevator,
                        arm,
                        drivetrain,
                        { IOLevel.Idle },
                        { selector.placementSide.asObject },
                        true
                    )
                )
        )
        .andThen(
            BuildingBlocks.goToHumanPlayerStation(drivetrain, arm, { Slider.far }, endAtAlignment = true)
                .deadlineWith(SetManipulatorSpeed(manipulator, .8))
                .deadlineWith(
                    SetSubsystemPosition(
                        elevator,
                        arm,
                        drivetrain,
                        { IOLevel.HumanPlayerSlider },
                        { selector.placementSide.asObject },
                        true
                    )
                )
        )
        .andThen(
            SetSubsystemPosition(
                elevator, arm,
                drivetrain,
                { IOLevel.HumanPlayerSlider },
                { selector.placementSide.asObject },
                true
            )
                .andThen(
                    BuildingBlocks.goToHumanPlayerStation(drivetrain, arm, { Slider.far }, endAtAlignment = false)
                )
                .andThen(WaitCommand(0.5))
                .deadlineWith(
                    SetManipulatorSpeed(manipulator, 1.0)
                )
        )
        .andThen(
            BuildingBlocks.leavePickupZone(drivetrain, arm)
                .deadlineWith(
                    SetSubsystemPosition(
                        elevator,
                        arm,
                        drivetrain,
                        { IOLevel.Idle },
                        { selector.placementSide.asObject },
                        true
                    )
                )
                .alongWith(
                    InstantCommand({
                        nextGamePieceCallback(
                            when (selector.placementSide) {
                                PlacementSide.FarCone -> PlacementSide.CloseCone
                                PlacementSide.Cube -> PlacementSide.FarCone
                                PlacementSide.CloseCone -> PlacementSide.Cube
                            }.asObject
                        )
                    })
                )
        )
        .repeatedly()
        .beforeStarting(
            InstantCommand({
                nextGamePieceCallback(
                    when (selector.placementSide) {
                        PlacementSide.FarCone -> PlacementSide.CloseCone
                        PlacementSide.Cube -> PlacementSide.FarCone
                        PlacementSide.CloseCone -> PlacementSide.Cube
                    }.asObject
                )
            })
        )
}
