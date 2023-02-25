package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Intake

class deploy(
    var intake: Intake,
    var controlScheme: ControlScheme,
): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun execute() {
        if(controlScheme.intakeTest.asBoolean){
            intake.setIntakePosition(1.0)
        } else {
            intake.setIntakePosition(0.0)
        }
    }

}
