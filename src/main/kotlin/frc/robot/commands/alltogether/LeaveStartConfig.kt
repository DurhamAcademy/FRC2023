package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Arm
import frc.robot.utils.GamePiece
import kotlin.math.PI
import kotlin.math.abs

class LeaveStartConfig (
    val robotContainer: RobotContainer,
    val arm: Arm
) : CommandBase() {
        override fun execute() {
            SetSubsystemPosition(robotContainer, { IOLevel.StartingConfig }, { GamePiece.cone })
        }
        override fun end(interrupted: Boolean) {
            SetSubsystemPosition(robotContainer, { IOLevel.Idle }, { GamePiece.cone })
        }
        override fun isFinished(): Boolean = abs(arm.armPosition) < PI + degreesToRadians(5.0)
}