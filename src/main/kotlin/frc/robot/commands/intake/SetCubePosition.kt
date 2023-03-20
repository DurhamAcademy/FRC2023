package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.intake
import frc.robot.subsystems.Intake

class SetCubePosition(
    val intake: Intake
): CommandBase() {
    init {
        addRequirements(intake)
        SetConePosition(intake)
    }

    override fun execute() {
        intake.modeMotorPercentage = 1.0
    }

    override fun end(interrupted: Boolean) {
        intake.modeMotorPercentage = 0.0
    }

    override fun isFinished(): Boolean {
        //TODO CHANGE THIS VALUE TO WHAT IS THE ACTUAL VALUE
        return intake.modeMotorPosition >= 180.0
    }

}