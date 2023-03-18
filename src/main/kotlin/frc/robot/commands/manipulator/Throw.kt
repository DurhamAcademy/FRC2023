package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Manipulator
import frc.robot.utils.GamePiece
import frc.robot.utils.GamePiece.cone
import frc.robot.utils.GamePiece.cube
import frc.robot.utils.grid.PlacementLevel

class Throw(
    private val manipulator: Manipulator,
    val gamePiece: () -> GamePiece,
    val placementLevel: () -> PlacementLevel
) : CommandBase() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.motorPercentage = when (gamePiece()) {
            cone -> placementLevel().ioLevel.coneVelocity
            cube -> placementLevel().ioLevel.cubeVelocity
            else -> -0.15
        }
    }
}