package frc.robot.commands.pathing.building.blocks

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import edu.wpi.first.wpilibj2.command.Command
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
    fun leaveCommunityZone(
        drivetrain: Drivetrain,
        alliance: () -> DriverStation.Alliance = {Game.alliance}
    ): Command? {
        val clearUp = 4.675
        val clearDown = 1.169
        val exitPoint: () -> Double ={
            when(alliance()){
                Red -> 9.0
                Blue -> 6.7
                else -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        val middleX: () -> Double ={
            when(alliance()){
                Red -> 13.4
                Blue -> 2.5
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        val clearRoute: () -> Boolean = {
            drivetrain.estimatedPose2d.y < clearUp + .25 && drivetrain.estimatedPose2d.y > clearUp - .25 || drivetrain.estimatedPose2d.y > clearDown - .25 && drivetrain.estimatedPose2d.y < clearDown + .25
        }
        val placementX: () -> Double = {
            if(clearRoute()){
                exitPoint()
            }
            else{
                middleX()
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
            when(alliance()){
                Red -> 180.0
                Blue -> 0.0
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        return MoveToPosition(
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
        val upperYValue = 4.675
        val lowerYValue = 1.169
        val chargeLimit: () -> Double = { xCenter + (((robotLength / 2) + 4.8) * -alliance().xMul) }
        val isInGridZone: () -> Boolean = {
            when(alliance()){
                Red -> drivetrain.estimatedPose2d.x > chargeLimit()
                Blue -> drivetrain.estimatedPose2d.x < chargeLimit()
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        val placementX: () -> Double = { xCenter + (((robotLength / 2) + 5.2) * -alliance().xMul) }
        val placementY: () -> Double = {
            if(isInGridZone()) group.offset - side.offset
            else if(abs(upperYValue - drivetrain.estimatedPose2d.y) > abs(lowerYValue - drivetrain.estimatedPose2d.y)) lowerYValue
            else upperYValue
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