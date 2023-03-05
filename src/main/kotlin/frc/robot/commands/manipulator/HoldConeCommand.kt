package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Manipulator

class HoldConeCommand(
    private val manipulator: Manipulator
) : CommandBase() {
    init {
        addRequirements(manipulator)
    }

    override fun initialize() {
        manipulator.isOpen = false
    }

    override fun execute() {
        manipulator.motorPercentage = 0.1
    }

    override fun end(interrupted: Boolean) {
        manipulator.motorPercentage = 0.0
    }

    override fun isFinished(): Boolean {
        return false
    }
}