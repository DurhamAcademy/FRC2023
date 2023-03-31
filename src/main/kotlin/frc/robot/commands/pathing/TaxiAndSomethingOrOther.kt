package frc.robot.commands.pathing

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation2d.fromDegrees
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.kyberlib.command.Game
import frc.kyberlib.math.units.Pose2d
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.SetSubsystemPosition
import frc.robot.commands.balance.AutoBalance
import frc.robot.commands.manipulator.Throw
import frc.robot.commands.pathing.building.blocks.BuildingBlocks
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.goToPlacementPoint
import frc.robot.commands.pathing.building.blocks.BuildingBlocks.leaveCommunityZone
import frc.robot.utils.GamePiece
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide

class TaxiAndSomethingOrOther(
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
            .withTimeout(7.0)
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            if (Game.alliance == Blue) 6.1
                            else if (Game.alliance == Red) 10.5
                            else robotContainer.drivetrain.estimatedPose2d.x,
                            4.75,
                            if (Game.alliance == Blue) fromDegrees(.0)
                            else if (Game.alliance == Red) fromDegrees(180.0)
                            else robotContainer.drivetrain.estimatedPose2d.rotation,
                        )
                    },
                    maxPosSpeed = 2.0
                ).alongWith(
                    SetSubsystemPosition(
                        robotContainer.elevator, robotContainer.arm,robotContainer.drivetrain, {IOLevel.Idle},
                        {GamePiece.cone}, true
                    )
                )
            )
            .andThen(
                MoveToPosition(
                    robotContainer.drivetrain,
                    {_,_,_ ->
                        Pose2d(
                            robotContainer.drivetrain.estimatedPose2d.x,
                            robotContainer.drivetrain.estimatedPose2d.y,
                            if (Game.alliance == Blue) fromDegrees(180.0)
                            else if (Game.alliance == Red) fromDegrees(0.0)
                            else robotContainer.drivetrain.estimatedPose2d.rotation,
                        )
                    }
                )
            )

}
