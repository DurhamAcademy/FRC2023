package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.alltogether.IOLevel
import frc.robot.subsystems.Manipulator
import frc.robot.utils.GamePiece

class ManipulatorIO(
    private val manipulator: Manipulator,
    inline val gamePiece: () -> GamePiece,
    inline val placementLevel: () -> IOLevel
) : CommandBase() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.motorPercentage = when (gamePiece()) {
            GamePiece.cone -> placementLevel().coneVelocity
            GamePiece.cube -> placementLevel().cubeVelocity
            else -> (placementLevel().coneVelocity + placementLevel().cubeVelocity) / 2
        }
    }
}