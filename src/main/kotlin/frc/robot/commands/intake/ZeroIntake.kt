package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class ZeroIntake(
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

    override fun isFinished(): //TODO check if volatage gets high
}