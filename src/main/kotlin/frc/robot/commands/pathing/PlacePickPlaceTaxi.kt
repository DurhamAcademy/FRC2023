package frc.robot.commands.pathing

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToPlacementPoint
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.leaveCommunityZone
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.middleX
import frc.robot.constants.Field2dLayout
import frc.robot.constants.Field2dLayout.xCenter
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide
import kotlin.math.abs

class PlacePickPlaceTaxi(
    private val robotContainer: RobotContainer,
) : Auto {
    override fun getCommand(): Command =
        goToPlacementPoint(
            robotContainer.drivetrain,
            robotContainer.arm,
            IOLevel.High,
            PlacementGroup.Farthest,
            PlacementSide.FarCone
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
                    { PlacementLevel.Level3 }
                )
                    .withTimeout(0.75)
            ) // shoot cube
            .andThen(
                leaveCommunityZone(robotContainer.drivetrain,
                    robotContainer.arm,
                )
            ) // move to the other side of the field
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.Low },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to pick up cone on L1 middle
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else 0.0,
                            2.72,
                            Rotation2d.fromDegrees(180.0)
                        )
                    },
                    maxPosSpeed = 3.7
                )
            ) // move to the other side of the field
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.Low },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to pick up cone on L1 middle
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else 0.0,
                            2.72,
                            Rotation2d.fromDegrees(180.0)
                        )
                    },
                    maxPosSpeed = 3.7
                )
            ) // move to the other side of the field
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.Low },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to pick up cone on L1 middle
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else 0.0,
                            2.72,
                            Rotation2d.fromDegrees(180.0)
                        )
                    },
                    maxPosSpeed = 3.7
                )
            ) // move to the other side of the field
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.Low },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to pick up cone on L1 middle
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else 0.0,
                            2.72,
                            Rotation2d.fromDegrees(180.0)
                        )
                    },
                    maxPosSpeed = 3.7
                )
            ) // move to the other side of the field
            .andThen(
                SetSubsystemPosition(
                    robotContainer,
                    { IOLevel.Low },
                    { GamePiece.cone },
                    true
                )
            ) // get in position to pick up cone on L1 middle
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else 0.0,
                            2.72,
                            Rotation2d.fromDegrees(180.0)
                        )
                    },
                    maxPosSpeed = 3.7
                )
            ) // move to the other side of the field
}