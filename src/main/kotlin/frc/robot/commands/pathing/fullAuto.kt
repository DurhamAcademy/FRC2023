package frc.robot.commands.pathing

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.building.blocks.BuildingBlocks
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator
import frc.robot.utils.GamePiece
import frc.robot.utils.Slider
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide

fun fullAuto(drivetrain: Drivetrain, arm: Arm, elevator: Elevator, manipulator: Manipulator): Command {
    return BuildingBlocks.goToPlacementPoint(
        drivetrain,
        arm,
        { PlacementLevel.Level3.ioLevel },
        { PlacementGroup.Closest },
        { PlacementSide.Cube },
    ).deadlineWith(
        SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { GamePiece.cube }, false)
    ).andThen(
        SetSubsystemPosition(
            elevator, arm,
            { PlacementLevel.Level3.ioLevel },
            { GamePiece.cube },
            true
        )
    ).andThen(
        Throw(
            manipulator,
            { GamePiece.cube },
            { PlacementLevel.Level3 },
        ).withTimeout(0.25)
    )
        .andThen(
            BuildingBlocks.leaveCommunityZone(drivetrain, arm)
                .deadlineWith(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { GamePiece.cube }, true))
        ).andThen(
            BuildingBlocks.goToPickupZone(drivetrain, arm)
                .deadlineWith(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { GamePiece.cube }, true))
        )
        .andThen(
            BuildingBlocks.goToHumanPlayerStation(drivetrain, arm, { Slider.far }, endAtAlignment = true)
                .deadlineWith(SetManipulatorSpeed(manipulator, .8))
                .deadlineWith(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { GamePiece.cube }, true))
        )
        .andThen(
            SetSubsystemPosition(
                elevator, arm,
                { IOLevel.HumanPlayerSlider },
                { GamePiece.cube },
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
                .deadlineWith(SetSubsystemPosition(elevator, arm, { IOLevel.Idle }, { GamePiece.cube }, true))
        )
        .repeatedly()
}
