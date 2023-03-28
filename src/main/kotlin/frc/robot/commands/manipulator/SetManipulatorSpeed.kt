package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Manipulator

class SetManipulatorSpeed(private val manipulator: Manipulator, private val speed: Double) : CommandBase() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.motorPercentage = speed
    }

    override fun isFinished(): Boolean {
        return false
    }
}