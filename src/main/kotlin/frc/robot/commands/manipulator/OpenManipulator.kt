package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Manipulator

class OpenManipulator(private val manipulator: Manipulator):InstantCommand() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        manipulator.open= true
    }
}