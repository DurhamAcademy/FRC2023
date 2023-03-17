package frc.robot.commands.alltogether

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units.inchesToMeters

/**
 * This enum represents the positions the elevator and arm should go to when scoring at different levels
 *
 * @param elevatorHeight the height the elevator needs to go to in meters
 * @param armRotation the rotation the arm needs to be at as a Rotation2d
 */
enum class IOLevel(val elevatorHeight: Double, val armRotation: Rotation2d){
    HighCone(frc.robot.constants.elevator.limits.topLimit - inchesToMeters(2.0), Rotation2d.fromRadians(1.32)),
    HighCube(frc.robot.constants.elevator.limits.topLimit - inchesToMeters(2.0), Rotation2d.fromRadians(1.32)),
    MidCone(inchesToMeters(38.0), Rotation2d.fromRadians(1.32)),
    MidCube(inchesToMeters(38.0), Rotation2d.fromRadians(1.32)),
    LowCone(frc.robot.constants.elevator.limits.topLimit, Rotation2d.fromRadians(2.61)),
    LowCube(frc.robot.constants.elevator.limits.topLimit, Rotation2d.fromRadians(2.61)),
    HumanPlayerSlider(1.3 - inchesToMeters(11.0), Rotation2d.fromRadians(1.4)),
    Idle(frc.robot.constants.elevator.limits.bottomLimit, Rotation2d.fromRadians(0.0)),
    FloorIntake(frc.robot.constants.elevator.limits.bottomLimit + inchesToMeters(1.0), Rotation2d.fromRadians(1.81))
}