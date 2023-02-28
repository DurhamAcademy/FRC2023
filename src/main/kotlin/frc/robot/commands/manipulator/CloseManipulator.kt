package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Manipulator

class CloseManipulator(private val manipulator: Manipulator):InstantCommand() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.isOpen= false
        manipulator.motorPercentage = -0.02
    }
}