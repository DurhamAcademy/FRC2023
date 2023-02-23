package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Wrist

class SetWristAngle(
    private val wrist: Wrist,
) : CommandBase() {
    init {
        addRequirements(wrist)
    }

    override fun initialize() {
        wrist.setPosition(90.0)
    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return true
    }
}