package frc.robot.commands.pathing.building.blocks

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import frc.kyberlib.command.Game
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Field2dLayout
import frc.robot.constants.Field2dLayout.xCenter
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementSide
import frc.robot.utils.grid.PlacmentLevel
import java.security.InvalidParameterException
import java.util.function.BinaryOperator
import kotlin.math.PI
import kotlin.math.abs
import frc.robot.constants.RobotProportions.length as robotLength

object BuildingBlocks {
    fun LeaveCommunityZone(
        drivetrain: Drivetrain,
        alliance: DriverStation.Alliance = DriverStation.getAlliance()
    ){
        val clearUp = 0.0
        val clearDown = 0.0
        val exitPoint = 0.0
        val middleX = 0.0
        val clearRoute: () -> Boolean = {
            drivetrain.estimatedPose2d.y == clearUp || drivetrain.estimatedPose2d.y == clearDown
        }
        val placementX: () -> Double = {
            if(clearRoute()){
                exitPoint
            }
            else{
                middleX
            }
        }
        val placementY: () -> Double = {
            if (abs(clearUp - drivetrain.estimatedPose2d.y) > abs(clearDown - drivetrain.estimatedPose2d.y)){
                clearDown
            }
            else{
                clearUp
            }
            //TODO charge station
        }
        val rotationAlliance: () -> Double = {
            when(alliance){
                Red -> 180.0
                Blue -> 0.0
                else -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    Rotation2d.fromDegrees(rotationAlliance())
                )
            }
        )
    }
    fun GoToPlacementPoint(
        drivetrain: Drivetrain,
        level: PlacmentLevel,
        group: PlacementGroup,
        side: PlacementSide,
        alliance: () -> DriverStation.Alliance = {Game.alliance}
    ): MoveToPosition {
        val chargeLimit: () -> Double = { xCenter + (((robotLength / 2) + 4.8) * alliance().xMul) }
        val isInGridZone: () -> Boolean = {
            drivetrain.estimatedPose2d.x < chargeLimit()
        }
        val placementX: () -> Double = { xCenter + (((robotLength / 2) + 6.15) * alliance().xMul) }
        val placementY: () -> Double = {
            if(isInGridZone()) group.offset + side.offset
            else drivetrain.estimatedPose2d.y //TODO fix so it snaps to a position
        }
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    Rotation2d()
                )
            }
        )
    }
}
val DriverStation.Alliance.xMul
    get() = when (this) {
        Red -> Field2dLayout.Axes.Red.fieldOffsetMultiplier
        Blue -> Field2dLayout.Axes.Blue.fieldOffsetMultiplier
        Invalid -> throw InvalidParameterException("Alliance must be valid")
    }