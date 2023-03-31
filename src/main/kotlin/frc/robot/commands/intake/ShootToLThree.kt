package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece
import kotlin.math.PI

class ShootToLThree(
    val intake: Intake,
    val robotContainer: RobotContainer,
): CommandBase() {

    init{
        addRequirements(intake)
    }

    override fun execute(){
        //intake.setDeployAngle(-1.0)
        intake.intakePercentage = -0.15
        intake.setModeAngle(0.5)
        intake.intakePercentage = 1.0

    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }

}