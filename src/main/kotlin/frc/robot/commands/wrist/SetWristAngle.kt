package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Wrist

class SetWristAngle(
    private val wrist: Wrist,
    private val angle: Double = 0.0
) : CommandBase() {
    init {
        addRequirements(wrist)
    }

    override fun initialize() {
        wrist.setPosition(angle)
    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return true
    }
}