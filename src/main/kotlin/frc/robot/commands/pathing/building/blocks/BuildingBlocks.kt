package frc.robot.commands.pathing.building.blocks

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.command.Game
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.IOLevel.*
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Field2dLayout.xCenter
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.GamePiece
import frc.robot.utils.Slider
import frc.robot.utils.grid.FloorGamePiecePosition
import frc.robot.utils.grid.GridConstants.centerDistX
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementSide
import kotlin.math.*
import frc.robot.constants.RobotProportions.length as robotLength
import frc.robot.constants.RobotProportions.width as robotWidth

object BuildingBlocks {
    /**
     * Variables
     */
    val alliance: () -> DriverStation.Alliance = { Game.alliance }
    val exitFalseGoalPoint: () -> Double = {
        when (alliance()) {
            Red -> 10.53
            Blue -> 5.4
            else -> throw IllegalArgumentException("Alliance is not Blue or Red")
        }
    }
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

    /**
     * These functions check if the robt can rotate safely (ie the arm is up)
     */
    private fun safeRotation(armAngle: Double, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        if (armAngle.absoluteValue < 0.15) angle
        else drivetrainAngle
    private fun safeRotation(arm: Arm?, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        safeRotation(arm?.armPosition ?: 0.0, angle, drivetrainAngle)

    /**
     * Move to the game pieces from the floor
     */
    fun pickupObjectFromFloor(
        drivetrain: Drivetrain,
        arm: Arm,
        position: FloorGamePiecePosition,
    ): MoveToPosition {
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
            // if the robot is on the side of the field that the object is on
            if (abs(8 - drivetrain.estimatedPose2d.x) - .2 > abs(8 - exitPoint())) {
                if (clearRoute()) {
                    exitPoint()
                } else {
                    middleX()
                }
            } else {
                if (firstRun) {
                    posX = (8 + position.x * alliance().xMul) - cos(
                        atan2(
                            position.y - drivetrain.estimatedPose2d.y,
                            position.x - drivetrain.estimatedPose2d.x
                        )
                    )
                    posY = (position.y - sin(
                        atan2(
                            position.y - drivetrain.estimatedPose2d.y,
                            position.x - drivetrain.estimatedPose2d.x
                        )
                    ))
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
                    safeRotation(arm, rotation(), drivetrain.estimatedPose2d.rotation)
                )
            }
        )
    }

    /**
     * Leave the community zone
     */
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

    /**
     * Go to a specific node
     */
    fun goToPlacementPoint(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        level: () -> IOLevel,
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
            (drivetrain.estimatedPose2d.y - group().offset + side().offset).absoluteValue < 0.05
        }

        val altOffset = 0.2

        val placementX: () -> Double = {
            when (level()) {
                //TODO fill in values (replace 5.2)
                Low, Mid, High, HumanPlayerSlider ->
                    xCenter + ((-(robotLength / 2) + centerDistX -//4.46
                            if (!isClose()) altOffset
                            else level().offsetDistance ?: altOffset) * -alliance().xMul)

                else ->
                    throw IllegalArgumentException(
                        "Level is not Low, Mid, High, or HumanPlayerSlider"
                    )
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
                            Red -> Rotation2d.fromDegrees(180.0 - 2.0)
                            Blue -> Rotation2d.fromDegrees(0.0 - 2.0)
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
        level: IOLevel,
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

    fun goToHumanPlayerStation(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        slider: () -> Slider,
        gamePiece: () -> GamePiece,
        alliance: () -> DriverStation.Alliance = { Game.alliance },
        endAtAlignment: Boolean = false,
    ): Command {
        val placementY: () -> Double = {
            slider().fieldYValue
        }
        val isClose: (margin: Double?) -> Boolean = { margin ->
            (drivetrain.estimatedPose2d.y - placementY())
                .absoluteValue < (margin ?: 0.09)
        }

        val altOffset = 0.4

        val placementX: () -> Double = {
            xCenter + (((-robotLength / 2) + (16.2 - xCenter) + //4.46
                    -if (!isClose(null) || endAtAlignment) ((HumanPlayerSlider.offsetDistance ?: 0.0) + altOffset)
                    else (HumanPlayerSlider.offsetDistance ?: altOffset)) * alliance().xMul)
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
                            Red -> Rotation2d.fromDegrees(0.0 - 2.0)
                            Blue -> Rotation2d.fromDegrees(180.0 - 2.0)
                            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                        },
                        drivetrain.estimatedPose2d.rotation
                    )
                )
            }
        )
    }

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
        )
    }
}