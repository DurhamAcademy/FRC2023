package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.manipulator
import frc.robot.subsystems.Manipulator
import frc.robot.utils.GamePiece
import frc.robot.utils.GamePiece.*

class Throw (
    private val manipulator: Manipulator,
    val gamePiece: ()->GamePiece
) : CommandBase(){
    init {
        addRequirements(manipulator)
    }

    override fun execute(){
        manipulator.motorPercentage = when (gamePiece()) {
            cone -> -0.2
            cube -> -0.15
            unknown -> -0.15
            none -> -0.15
        }
    }
}