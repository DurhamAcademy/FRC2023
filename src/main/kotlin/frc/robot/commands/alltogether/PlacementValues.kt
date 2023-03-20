package frc.robot.commands.alltogether

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units.inchesToMeters
import frc.robot.constants.elevator.limits.bottomLimit
import frc.robot.constants.elevator.limits.topLimit

/**
 * This enum represents the positions the elevator and arm should go to when scoring at different levels
 *
 * @param coneElevatorHeight the height the elevator needs to go to in meters for a cone
 * @param coneArmRotation the rotation the arm needs to be at as a Rotation2d for a cone
 * @param cubeElevatorHeight same as coneElevatorHeight but for a cube
 * @param cubeArmRotation same as coneArmRotation but for a cube
 */
enum class IOLevel(
    val coneElevatorHeight: Double,
    val coneArmRotation: Rotation2d,
    val cubeElevatorHeight: Double,
    val cubeArmRotation: Rotation2d,
    val cubeVelocity: Double,
    val coneVelocity: Double,
    val offsetDistance: Double? = null
) {
    High(
        topLimit - inchesToMeters(4.0),
        Rotation2d.fromRadians(1.2),
        .85,
        Rotation2d.fromRadians(1.32),
        -0.15, -0.15,
        -0.075
    ),
    Mid(
        .7,
        Rotation2d.fromRadians(1.175),
        .85,
        Rotation2d.fromRadians(1.32),
        -0.15, -0.10,
        .2668
    ),
    Low(
        //for the height here is 0.88
        1.0,
        Rotation2d.fromRadians(2.61),
        0.88,
        Rotation2d.fromRadians(2.61),
        -0.15, -0.15,
        0.5
    ),
    HumanPlayerSlider(
        1.3 - inchesToMeters(11.0 + 3.0),
        Rotation2d.fromRadians(1.4),
        1.3 - inchesToMeters(11.0 + 2.75),
        Rotation2d.fromRadians(1.4),
        0.40, 1.0,
        inchesToMeters(24.0)
    ),
    Idle(
        bottomLimit,
        Rotation2d.fromRadians(0.0),
        bottomLimit,
        Rotation2d.fromRadians(0.0),
        -0.15, -0.15
    ),
    FloorIntake(
        bottomLimit + inchesToMeters(1.0),
        Rotation2d.fromRadians(1.81),
        bottomLimit + inchesToMeters(1.0),
        Rotation2d.fromRadians(1.81),
        -0.15, -0.15
    ),
    StartingConfig(
        topLimit,
        Rotation2d.fromDegrees(220.0),
        topLimit,
        Rotation2d.fromDegrees(220.0),
        0.0, 0.0
    )
}