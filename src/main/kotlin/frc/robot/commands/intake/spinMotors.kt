package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake
import frc.robot.controls.ControlScheme
class spinMotors(
    var intake: Intake,
    var controlScheme: ControlScheme,
): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun execute() {
        if(controlScheme.intakeTest.asBoolean) {
            intake.spinIntakeMotor(1.0)
        }
    }
}