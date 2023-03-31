package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake
import kotlin.math.PI

class IntakeEject(
    val intake: Intake,
    /**
     * If true, the intake will eject the ojbect and move to the deploy position at the same time.
     * If false, the intake will only move to the deploy position.
     */
    val eject: Boolean = true
) : CommandBase() {

    override fun execute() {
        intake.modeVoltage = -3.0
        intake.setDeployAngle(PI * 2 - 0.8)
        if (intake.deployPID.atGoal() && eject)
            intake.intakePercentage = 0.35
        else
            intake.intakePercentage = -0.1
    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }
}