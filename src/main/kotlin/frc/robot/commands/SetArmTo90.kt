package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm

class SetArmTo90(
    private val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(arm)
    }

    override fun initialize() {
        arm.setArmPosition(90.0)
    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) {
    }

    override fun isFinished(): Boolean {
        return true
    }
}