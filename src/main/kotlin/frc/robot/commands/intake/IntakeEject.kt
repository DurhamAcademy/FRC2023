package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Intake
import kotlin.math.PI

class IntakeEject(
    val intake: Intake,
    val robotContainer: RobotContainer,
) : CommandBase(){

    override fun execute(){
        intake.modeVoltage = -3.0
        intake.setDeployAngle(PI * 2 - 0.8)
        intake.intakePercentage = 0.25
    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }

}