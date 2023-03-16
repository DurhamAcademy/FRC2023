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
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.grid.FloorGamePiecePosition
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementSide
import frc.robot.utils.grid.PlacmentLevel
import java.security.InvalidParameterException
import kotlin.math.*
import frc.robot.constants.RobotProportions.length as robotLength

object BuildingBlocks {
    val alliance: () -> DriverStation.Alliance = {Game.alliance}
    val exitPoint: () -> Double ={
        when(alliance()){
            Red -> 10.53
            Blue -> 5.4
            else -> throw IllegalArgumentException("Alliance is not Blue or Red")
        }
    }
    val clearUp = 4.675 //Y value above charge station
    val clearDown = 1.169//Y value below charge station
    val middleX: () -> Double ={
        when(alliance()){
            Red -> 12.0
            Blue -> 2.5
            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
        }
    }
    fun pickupObjectFromFloor(
        drivetrain: Drivetrain,
        position: FloorGamePiecePosition
    ): MoveToPosition{
        var firstRun = true
        var posX = 0.0
        var posY = 0.0
        val clearRoute: () -> Boolean = {
            drivetrain.estimatedPose2d.y < clearUp + .25 && drivetrain.estimatedPose2d.y > clearUp - .25 || drivetrain.estimatedPose2d.y > clearDown - .25 && drivetrain.estimatedPose2d.y < clearDown + .25
        }
        val placementY: () -> Double = {
            if(abs(8 - drivetrain.estimatedPose2d.x) > abs(8 - exitPoint())) {
                if (abs(clearUp - drivetrain.estimatedPose2d.y) > abs(clearDown - drivetrain.estimatedPose2d.y)) {
                    clearDown
                } else {
                    clearUp
                }
                //TODO charge station
            }
            else{
                posY
            }
        }
        val placementX: () -> Double = {
            if(abs(8 - drivetrain.estimatedPose2d.x) - .2> abs(8 - exitPoint())) {
                if (clearRoute()) {
                    exitPoint()
                } else {
                    middleX()
                }
            }
            else{
                if(firstRun){
                    posX = when(alliance()){
                        Red -> (8 + position.x) - cos(atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x))
                        Blue -> (8 - position.x) - cos(atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x))
                        Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                    }
                    posY = when(alliance()){
                        Red -> (position.y - sin(atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x)))
                        Blue -> (position.y - sin(atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x)))
                        Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                    }
                    firstRun = false
                }
                posX
            }
        }
        val closestRightAngle : Double = round(drivetrain.estimatedPose2d.rotation.degrees/90) * 90
        val rotation: () -> Rotation2d = {
            if(abs(8 - drivetrain.estimatedPose2d.x) > abs(8 - exitPoint())){
                Rotation2d.fromDegrees(closestRightAngle)
            }
            else{
                Rotation2d.fromDegrees(
                    -atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x)
                )
            }
        }
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    rotation()
                )
            }
        )
    }
    fun leaveCommunityZone(
        drivetrain: Drivetrain,
        arm: Arm,
        ): Command? {
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
        val rotationAlliance: () -> Rotation2d = {
            if(arm.armPosition.absoluteValue < 0.5){
                when(alliance()){
                    Red -> Rotation2d.fromDegrees(0.0)
                    Blue -> Rotation2d.fromDegrees(180.0)
                    Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                }
            }
            else{
                drivetrain.estimatedPose2d.rotation
            }
        }
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    rotationAlliance()
                )
            }
        )
    }
    fun goToPlacementPoint(
        drivetrain: Drivetrain,
        level: PlacmentLevel,
        group: PlacementGroup,
        side: PlacementSide,
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
        val placementX: () -> Double = {
            when(level){
                //TODO fill in values (replace 5.2)
                PlacmentLevel.Level1 -> xCenter + (((robotLength / 2) + 5.2) * -alliance().xMul)
                PlacmentLevel.Level2 -> xCenter + (((robotLength / 2) + 5.2) * -alliance().xMul)
                PlacmentLevel.Level3 -> xCenter + (((robotLength / 2) + 5.3) * -alliance().xMul)
            }
        }
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