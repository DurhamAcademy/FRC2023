package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Manipulator
import frc.robot.utils.GamePiece
import frc.robot.utils.GamePiece.*

class Throw (
    private val manipulator: Manipulator,
    private val speed:Double,
    val gamePiece: () -> GamePiece
) : CommandBase(){
    init {
        addRequirements(manipulator)
    }

    override fun execute(){
        when(gamePiece()) {
            cone -> SetManipulatorSpeed(manipulator, -0.2)
            cube -> SetManipulatorSpeed(manipulator, -0.15)
            unknown -> SetManipulatorSpeed(manipulator, -0.15)
            none -> SetManipulatorSpeed(manipulator, -0.15)
        }
    }
}