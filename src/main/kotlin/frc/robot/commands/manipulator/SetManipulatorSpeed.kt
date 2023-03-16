package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Manipulator

class SetManipulatorSpeed(private val manipulator: Manipulator, private val speed:Double):InstantCommand() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.motorPercentage = speed
    }
}