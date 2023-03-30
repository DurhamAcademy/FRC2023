package frc.robot.commands.pathing.building.blocks

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.*
import edu.wpi.first.wpilibj2.command.Command
import frc.kyberlib.command.Game
import frc.robot.commands.alltogether.IOLevel
import frc.robot.commands.alltogether.IOLevel.*
import frc.robot.commands.pathing.MoveToPosition
import frc.robot.constants.Field2dLayout.xCenter
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.utils.Slider
import frc.robot.utils.grid.FloorGamePiecePosition
import frc.robot.utils.grid.GridConstants.centerDistX
import frc.robot.utils.grid.PlacementGroup
import frc.robot.utils.grid.PlacementSide
import frc.robot.utils.xMul
import kotlin.math.*
import frc.robot.constants.RobotProportions.length as robotLength
import frc.robot.constants.RobotProportions.width as robotWidth
import frc.robot.constants.drivetrain as drivetrainConstraints

object BuildingBlocks {
    val robotDiagRadius = hypot(robotLength / 2.0, robotWidth / 2.0) + 0.1

    /**
     * Variables
     */
    val exitFalseGoalPoint = { alliance: Alliance ->
        xCenter + ((2.15 - robotDiagRadius) * alliance.xMul)
    }
    val clearUp = 4.0 + (robotLength / 2.0) + 0.25 //Y value above charge station
    val clearDown = 1.5 - (robotLength / 2.0) - 0.25 //Y value below charge station
    val exitPoint = { alliance: Alliance ->
        xCenter + ((3.3 - robotDiagRadius) * -alliance.xMul)
    }
    val middleX = { alliance: Alliance ->
        xCenter + (6.0 * -alliance.xMul)
    }

    /**
     * These functions return the direction to rotate that will get from a start
     * rotation to an end rotation with the least amount of rotation.
     * @param start the start rotation
     * @param end the end rotation
     * @return true if the shortest rotation is an increase in angle (on the
     * unit circle) and false if the shortest rotation is a decrease in angle.
     */
    fun shortestRotationDirection(start: Rotation2d, end: Rotation2d): Boolean {
        // Get the difference between the two angles
        val difference = end - start
        // if the difference is less than 180 degrees, return true
        return MathUtil.angleModulus(difference.radians.absoluteValue) < 0.0
    }

    /**
     * These functions check if the robt can rotate safely (ie the arm is up)
     */
    private fun safeRotation(armAngle: Double, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        if (armAngle.absoluteValue < 0.15) angle
        else drivetrainAngle

    //                if (!shortestRotationDirection(drivetrainAngle, angle))
//                    Rotation2d(PI/4)
//                else
//                    Rotation2d(-PI/4)
    fun safeRotation(arm: Arm?, angle: Rotation2d, drivetrainAngle: Rotation2d) =
        safeRotation(arm?.armPosition ?: 0.0, angle, drivetrainAngle)

    /**
     * Move to the game pieces from the floor
     */
    fun pickupObjectFromFloor(
        drivetrain: Drivetrain,
        arm: Arm,
        position: FloorGamePiecePosition,
        alliance: () -> Alliance
    ): MoveToPosition {
        var firstRun = true
        var posX = 0.0
        var posY = 0.0
        val clearRoute: () -> Boolean = {
            drivetrain.estimatedPose2d.y < clearUp + .25 && drivetrain.estimatedPose2d.y > clearUp - .25 || drivetrain.estimatedPose2d.y > clearDown - .25 && drivetrain.estimatedPose2d.y < clearDown + .25
        }
        val placementY: () -> Double = {
            if (abs(8 - drivetrain.estimatedPose2d.x) > abs(8 - exitPoint(alliance()))) {
                if (abs(clearUp - drivetrain.estimatedPose2d.y) > abs(clearDown - drivetrain.estimatedPose2d.y)) {
                    clearDown
                } else {
                    clearUp
                }
                //TODO charge station
            } else {
                posY
            }
        }
        val placementX: () -> Double = {
            // if the robot is on the side of the field that the object is on
            if (abs(8 - drivetrain.estimatedPose2d.x) - .2 > abs(8 - exitPoint(alliance()))) {
                if (clearRoute()) {
                    exitPoint(alliance())
                } else {
                    //
                    middleX(alliance())
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
        val closestRightAngle: Double = round(drivetrain.estimatedPose2d.rotation.degrees / 90) * 90
        val rotation: () -> Rotation2d = {
            if (abs(8 - drivetrain.estimatedPose2d.x) > abs(8 - exitPoint(alliance()))) {
                Rotation2d.fromDegrees(closestRightAngle)
            } else {
                Rotation2d.fromDegrees(
                    -atan2(position.y - drivetrain.estimatedPose2d.y, position.x - drivetrain.estimatedPose2d.x)
                )
            }
        }
        return MoveToPosition(
            drivetrain,
            { _, _, _ ->
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
        alliance: () -> Alliance = { Game.alliance }
    ): Command {
        val hasExited: () -> Boolean = {
            when (alliance()) {
                Red -> drivetrain.estimatedPose2d.x < exitPoint(alliance())
                Blue -> drivetrain.estimatedPose2d.x > exitPoint(alliance())
                else -> true//fixme: NOOO
            }

        }
        val clearRoute: () -> Boolean = {
            (
                    (
                            drivetrain.estimatedPose2d.y < clearUp + .25
                            ).and(
                            drivetrain.estimatedPose2d.y > clearUp - .25
                        )
                    ).or(
                    (
                            drivetrain.estimatedPose2d.y > clearDown - .25
                            ).and(
                            drivetrain.estimatedPose2d.y < clearDown + .25
                        )
                )
        }
        val placementX: () -> Double = {
            if (clearRoute()) {
                exitFalseGoalPoint(alliance())
            } else {
                middleX(alliance())
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
            { xPID, _, _ ->
                if (drivetrain.estimatedPose2d.y < clearDown + .25 && (drivetrain.estimatedPose2d.x - (xCenter + (4.27 * -alliance().xMul))).absoluteValue < (robotLength / 2.0) + 0.25) {
                    xPID.setConstraints(
                        TrapezoidProfile.Constraints(
                            drivetrainConstraints.maxAutonomousVelocity,
                            drivetrainConstraints.maxAutonomousAcceleration
                        )
                    )
                } else {
                    xPID.setConstraints(
                        TrapezoidProfile.Constraints(
                            drivetrainConstraints.maxAutonomousVelocity,
                            drivetrainConstraints.maxAutonomousAcceleration
                        )
                    )
                }
                return@MoveToPosition Pose2d(
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
    inline fun goToPlacementPoint(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        crossinline level: () -> IOLevel,
        crossinline group: () -> PlacementGroup,
        crossinline side: () -> PlacementSide,
        crossinline alliance: () -> Alliance = { Game.alliance },
    ): Command {
        val correctStartingPos: () -> Boolean = {
            when (alliance()) {
                Red -> drivetrain.estimatedPose2d.x > 8
                Blue -> drivetrain.estimatedPose2d.x < 8
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
        val upperYValue = clearUp
        val lowerYValue = clearDown
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
//        if(correctStartingPos()){
        return MoveToPosition(
            drivetrain,
            { _, _, _ ->
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
//        }
//        else return InstantCommand()
    }

    inline fun goToPlacementPoint(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        level: IOLevel,
        group: PlacementGroup,
        side: PlacementSide,
        crossinline alliance: () -> Alliance = { Game.alliance },
    ): Command =
        goToPlacementPoint(
            drivetrain,
            arm,
            { level },
            { group },
            { side },
            alliance
        )

    inline fun goToHumanPlayerStation(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        crossinline slider: () -> Slider,
        crossinline alliance: () -> Alliance = { Game.alliance },
        endAtAlignment: Boolean = false,
    ): Command {
        val correctStartingPos: () -> Boolean = {
            when (alliance()) {
                Red -> drivetrain.estimatedPose2d.x < 8
                Blue -> drivetrain.estimatedPose2d.x > 8
                Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
            }
        }
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
//        if(correctStartingPos()){
        return MoveToPosition(
            drivetrain,
            { xPid, yPid, rotPid ->
                if (arm == null) Unit
                else if (arm.armPosition.absoluteValue > 0.5) {
                    rotPid.setConstraints(
                        TrapezoidProfile.Constraints(
                            drivetrainConstraints.maxAutonomousAngularVelocity,
                            drivetrainConstraints.maxAutonomousAngularAcceleration
                        )
                    )
                } else {
                    rotPid.setConstraints(
                        TrapezoidProfile.Constraints(
                            drivetrainConstraints.maxAutonomousAngularVelocity,
                            drivetrainConstraints.maxAutonomousAcceleration
                        )
                    )
                }
                Pose2d(
                    placementX(),
                    placementY(),
                    when (alliance()) {
                        Red -> Rotation2d.fromDegrees(0.0 - 2.0)
                        Blue -> Rotation2d.fromDegrees(180.0 - 2.0)
                        Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                    }
                )
            }
        )
//        }
//        else return InstantCommand()
    }

    inline fun goToPickupZone(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        crossinline alliance: () -> Alliance = { Game.alliance },
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
                xCenter + (((3.0) - (robotLength / 2.0)) * alliance().xMul)
            else
                xCenter + (((3.0) - (robotLength / 2.0)) * alliance().xMul)
        }
        val placementY = midCommunityZoneLimit + robotWidth / 2.0
        return MoveToPosition(
            drivetrain,
            { _, _, _ ->
                Pose2d(
                    placementX(),
                    placementY,
                    safeRotation(
                        arm,
                        when (alliance()) {
                            Red -> Rotation2d.fromDegrees(0.0)
                            Blue -> Rotation2d.fromDegrees(180.0)
                            Invalid -> throw IllegalArgumentException("Alliance is not Blue or Red")
                        },
                        drivetrain.estimatedPose2d.rotation
                    )
                )
            }
        )
            .until(isInCommunityZone)
    }

    inline fun leavePickupZone(
        drivetrain: Drivetrain,
        arm: Arm? = null,
        crossinline alliance: () -> Alliance = { Game.alliance },
    ): Command {
        val hasLeftZone: () -> Boolean = {
            (((drivetrain.estimatedPose2d.x - xCenter).absoluteValue
                    < (4.0 - (robotLength / 2.0))
                    ) && (drivetrain.estimatedPose2d.y < (5.3 - (robotWidth / 2.0))))
                    || (drivetrain.estimatedPose2d.y < (5.3 - (robotWidth / 2.0)))
        }
        val x: () -> Double = {
            if (drivetrain.estimatedPose2d.y < (5.3 - (robotWidth / 2.0)))
                xCenter + (((robotLength / 2.0) + (5.22)) * -alliance().xMul)
            else xCenter + ((4.3 - (robotLength / 2.0)) * -alliance().xMul)
        }
        val y: () -> Double = {
            if ((drivetrain.estimatedPose2d.x - xCenter).absoluteValue < (4.73 - (robotLength / 2.0)))
                5.0 - (robotWidth / 2.0)
            else drivetrain.estimatedPose2d.y
        }
        return MoveToPosition(
            drivetrain,
            { _, _, _ ->
                Pose2d(
                    x(),
                    y(),
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
            .until { hasLeftZone() }
    }
}