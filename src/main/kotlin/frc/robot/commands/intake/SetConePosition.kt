package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class SetConePosition(
    val intake: Intake
) : CommandBase() {
    init{
        addRequirements(intake)
    }
    override fun execute() {
        intake.systemMotorPercentage = -1.0
    }

    override fun end(interrupted: Boolean) {
        intake.systemMotorPercentage = 0.0
        //zero
    }

    override fun isFinished(): Boolean {
        return intake.limitSwitchPressed
    }
}
