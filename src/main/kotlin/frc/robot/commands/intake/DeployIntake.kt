package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.constants.intake
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece
import kotlin.math.PI

class DeployIntake(
    val intake: Intake,
    val robotContainer: RobotContainer
): CommandBase() {

    init{
        addRequirements(intake)
    }

    override fun execute(){
        when (GamePiece.cone) {
            GamePiece.cone -> {
                intake.setDeployAngle(PI * 2 - 1.4)
                intake.setModeAngle(0.0)
            }
            GamePiece.cube -> {
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(2.45)
            }
            else -> {
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(2.45)
            }
        }
        intake.intakePercentage = -0.5

    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }

}