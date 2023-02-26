package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm

class SetArmTo90(
    private val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(arm)
    }

    override fun initialize() {
        arm.setArmPosition(arm.dashSetpoint.getDouble(0.0))
    }

    override fun execute() {
        arm.setArmPosition(arm.dashSetpoint.getDouble(0.0))
    }

    override fun end(interrupted: Boolean) {
        arm.armSetpoint = null
    }

    override fun isFinished(): Boolean {
        return false
    }
}