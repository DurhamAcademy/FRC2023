package frc.robot.commands.alltogether

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units.inchesToMeters

/**
 * This enum represents the positions the elevator and arm should go to when scoring at different levels
 *
 * @param elevatorHeight the height the elevator needs to go to in meters
 * @param armRotation the rotation the arm needs to be at as a Rotation2d
 */
enum class IOLevel(val coneElevatorHeight: Double, val coneArmRotation: Rotation2d, val cubeElevatorHeight: Double, val cubeArmRotation: Rotation2d){
    High(frc.robot.constants.elevator.limits.topLimit - inchesToMeters(2.0), Rotation2d.fromRadians(1.32), frc.robot.constants.elevator.limits.topLimit - inchesToMeters(2.0), Rotation2d.fromRadians(1.32)),
    Mid(inchesToMeters(38.0), Rotation2d.fromRadians(1.32),inchesToMeters(38.0), Rotation2d.fromRadians(1.32)),
    Low(frc.robot.constants.elevator.limits.topLimit, Rotation2d.fromRadians(2.61),frc.robot.constants.elevator.limits.topLimit, Rotation2d.fromRadians(2.61)),
    HumanPlayerSlider(1.3 - inchesToMeters(11.0), Rotation2d.fromRadians(1.4), 1.3 - inchesToMeters(11.0), Rotation2d.fromRadians(1.4)),
    Idle(frc.robot.constants.elevator.limits.bottomLimit, Rotation2d.fromRadians(0.0), frc.robot.constants.elevator.limits.bottomLimit, Rotation2d.fromRadians(0.0)),
    FloorIntake(frc.robot.constants.elevator.limits.bottomLimit + inchesToMeters(1.0), Rotation2d.fromRadians(1.81), frc.robot.constants.elevator.limits.bottomLimit + inchesToMeters(1.0), Rotation2d.fromRadians(1.81))
}