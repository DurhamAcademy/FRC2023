package frc.robot.commands.manipulator

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Manipulator

class MaintainGamepiece(private val manipulator: Manipulator): InstantCommand() {
    init {
        addRequirements(manipulator)
    }

    override fun execute() {
        SetManipulatorSpeed(manipulator, 0.1)
    }
}