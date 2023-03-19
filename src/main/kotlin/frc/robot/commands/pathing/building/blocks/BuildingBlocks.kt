package frc.robot.commands.pathing.building.blocks

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.command.Game
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Field2dLayout.xCenter
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.grid.FloorGamePiecePosition
import frc.robot.utils.grid.GridConstants.centerDistX
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementLevel
import frc.robot.utils.grid.PlacementSide
import kotlin.math.*
import frc.robot.constants.RobotProportions.length as robotLength
import frc.robot.constants.RobotProportions.width as robotWidth

object BuildingBlocks {
    val alliance: () -> DriverStation.Alliance = { Game.alliance }
    val floorIntakeAngle = IOLevel.FloorIntake.coneArmRotation.degrees //TODO revisit to make work if cone and cube rotation become different
    val exitFalseGoalPoint: () -> Double = {
        when (alliance()) {
            Red -> 10.53
            Blue -> 5.4
            else -> throw IllegalArgumentException("Alliance is not Blue or Red")
        }
    }
    val armLength = frc.robot.constants.arm.length
    val clearUp = 4.0 //Y value above charge station
    val clearDown = 1.5 //Y value below charge station
    val exitPoint: () -> Double = {
        xCenter + (((robotLength / 2.0) + 3.0) * -alliance().xMul)
    }
    val middleX: () -> Double = {
        when (alliance()) {
            Red -> 12.0
            Blue -> 2.5
            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
        }
    }

    private fun safeRotation(armAngle: Double, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        if (armAngle.absoluteValue < 0.15) angle
        else Rotation2d.fromRadians(
            max(
                min(
                    drivetrainAngle.radians - angle.radians,
                    Rotation2d.fromDegrees(5.0).radians
                ), Rotation2d.fromDegrees(-5.0).radians
            )
        ) + drivetrainAngle

    private fun safeRotation(arm: Arm?, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        safeRotation(arm?.armPosition ?: 0.0, angle, drivetrainAngle)

    fun pickupObjectFromFloor(
        drivetrain: Drivetrain,
        arm: Arm,
        position: FloorGamePiecePosition,
    ): MoveToPosition {
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
                position.y
            }
        }
        val placementX: () -> Double = {
            // if the robot is on the side of the field that the object is on
            if ((abs(8 - drivetrain.estimatedPose2d.x) - .2) > abs(8 - exitPoint())) {
                if (clearRoute()) {
                    exitPoint()
                } else {
                    middleX()
                }
            } else if (abs(position.x - drivetrain.estimatedPose2d.x) < .2 && abs(position.y - drivetrain.estimatedPose2d.y) < .2) {
                when (alliance()) {
                    Red -> 8 + position.x + armLength + .2
                    Blue -> 8 - position.x - armLength - .2
                    Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                }
            } else if (((arm.armPosition - .25) < floorIntakeAngle) && ((arm.armPosition + .25) > floorIntakeAngle)) {
                when (alliance()) {
                    Red -> (8 + position.x + armLength) - .5
                    Blue -> (8 - position.x - armLength) + .5
                    Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                }
            } else {
                drivetrain.estimatedPose2d.x
            }
        }
        val closestRightAngle : Double = round(drivetrain.estimatedPose2d.rotation.degrees/90) * 90
        val rotation: () -> Rotation2d = {
            when (alliance()) {
                Red -> Rotation2d.fromDegrees(0.0)
                Blue -> Rotation2d.fromDegrees(180.0)
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    safeRotation(arm, rotation(), drivetrain.estimatedPose2d.rotation)
                )
            }
        )
    }
    fun leaveCommunityZone(
        drivetrain: Drivetrain,
        arm: Arm,
        ): Command? {
        val hasExited: () -> Boolean = {
            (exitPoint() - drivetrain.estimatedPose2d.x).absoluteValue < 0.25
        }
        val clearRoute: () -> Boolean = {
            drivetrain.estimatedPose2d.y < clearUp + .25 && drivetrain.estimatedPose2d.y > clearUp - .25 || drivetrain.estimatedPose2d.y > clearDown - .25 && drivetrain.estimatedPose2d.y < clearDown + .25
        }
        val placementX: () -> Double = {
            if (clearRoute()) {
                exitFalseGoalPoint()
            } else {
                middleX()
            }
        }
        val placementY: () -> Double = {
            if (abs(clearUp - drivetrain.estimatedPose2d.y) > abs(clearDown - drivetrain.estimatedPose2d.y)) {
                clearDown
            } else {
                clearUp
            }
            //TODO charge station
        }
        val rotationAlliance: () -> Rotation2d = {
            if (arm.armPosition.absoluteValue < 0.5) {
                when (alliance()) {
                    Red -> Rotation2d.fromDegrees(0.0)
                    Blue -> Rotation2d.fromDegrees(180.0)
                    Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                }
            } else {
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
            .until {
                hasExited()
            }
    }

    fun goToPlacementPoint(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        level: () -> PlacementLevel,
        group: () -> PlacementGroup,
        side: () -> PlacementSide,
        alliance: () -> DriverStation.Alliance = { Game.alliance },
    ): Command {
        val upperYValue = 4.675
        val lowerYValue = 1.169
        val chargeLimit: () -> Double = { xCenter + (((robotLength / 2.0) + 5.38) * -alliance().xMul) }
        val isInGridZone: () -> Boolean = {
            when (alliance()) {
                Red -> drivetrain.estimatedPose2d.x > chargeLimit()
                Blue -> drivetrain.estimatedPose2d.x < chargeLimit()
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        val isClose: () -> Boolean = {
            (drivetrain.estimatedPose2d.y - group().offset + side().offset).absoluteValue < 0.1
        }

        val placementX: () -> Double = {
            when (level()) {
                //TODO fill in values (replace 5.2)
                PlacementLevel.Level1 ->
                    xCenter + ((-(robotLength / 2) + centerDistX -//4.46
                            0.5) * -alliance().xMul)

                PlacementLevel.Level2 ->
                    xCenter + ((-(robotLength / 2) + centerDistX -
                            if (!isClose()) 0.1
                            else 0.0) * -alliance().xMul)

                PlacementLevel.Level3 ->
                    xCenter + ((-(robotLength / 2) + centerDistX -
                            if (!isClose()) 0.1
                            else 0.0) * -alliance().xMul)
            }
        }
        val placementY: () -> Double = {
            if (isInGridZone()) group().offset - side().offset
            else if (abs(upperYValue - drivetrain.estimatedPose2d.y) > abs(lowerYValue - drivetrain.estimatedPose2d.y)) lowerYValue
            else upperYValue
        }
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY(),
                    safeRotation(
                        arm,
                        when (alliance()) {
                            Red -> Rotation2d.fromDegrees(180.0)
                            Blue -> Rotation2d.fromDegrees(0.0)
                            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                        },
                        drivetrain.estimatedPose2d.rotation
                    )
                )
            }
        )
    }

    fun goToPlacementPoint(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        level: PlacementLevel,
        group: PlacementGroup,
        side: PlacementSide,
        alliance: () -> DriverStation.Alliance = { Game.alliance },
    ): Command =
        goToPlacementPoint(
            drivetrain,
            arm,
            { level },
            { group },
            { side },
            alliance
        )

    fun goToPickupZone(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        alliance: () -> DriverStation.Alliance = { Game.alliance },
    ): Command {
        val bottomCommunityZoneLimit = 5.75
        val midCommunityZoneLimit = 6.75
        val bottomStartingX: () -> Double = { xCenter + (((robotLength / 2.0) + (-5.15)) * -alliance().xMul) }
        val midStartingX: () -> Double = { xCenter + (((robotLength / 2.0) + (-1.825)) * -alliance().xMul) }
        val isInCommunityZone: () -> Boolean = {
            ((drivetrain.estimatedPose2d.y > bottomCommunityZoneLimit)
                    && (drivetrain.estimatedPose2d.x - bottomStartingX()) * -alliance().xMul > 0)
                    || ((drivetrain.estimatedPose2d.y > midCommunityZoneLimit)
                    && (drivetrain.estimatedPose2d.x - midStartingX()) * -alliance().xMul > 0)
        }
        val placementX: () -> Double = {
            if (isInCommunityZone())
                xCenter + (((robotLength / 2.0) + (-5.22)) * -alliance().xMul)
            else
                xCenter + (((robotLength / 2.0) + (-6.0)) * -alliance().xMul)
        }
        val placementY = midCommunityZoneLimit + robotWidth / 2.0
        return MoveToPosition(
            drivetrain,
            {
                Pose2d(
                    placementX(),
                    placementY,
                    safeRotation(
                        arm,
                        when (alliance()) {
                            Red -> Rotation2d.fromDegrees(180.0)
                            Blue -> Rotation2d.fromDegrees(0.0)
                            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                        },
                        drivetrain.estimatedPose2d.rotation
                    )
                )
            }
        ).until {
            isInCommunityZone()
        }
    }
}