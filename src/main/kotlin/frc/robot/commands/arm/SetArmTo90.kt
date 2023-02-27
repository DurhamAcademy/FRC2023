package frc.robot.commands.arm

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm

class SetArmToAngle(
    private val arm: Arm,
    val angle: Double
) : CommandBase() {
    init {
        addRequirements(arm)
    }

    override fun initialize() {
        arm.reset()
    }

    override fun execute() {
        arm.setArmPosition(angle)
    }

    override fun end(interrupted: Boolean) {
        arm.armSetpoint = null
    }

    override fun isFinished(): Boolean {
        return false
    }
}