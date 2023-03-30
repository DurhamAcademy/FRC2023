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
            //SWAPPED
            GamePiece.cone -> {
                intake.modeVoltage = -3.0
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(1.95)

            }
            GamePiece.cube -> {
                intake.modeVoltage = -3.0
                intake.setDeployAngle(PI * 2 - 1.4)
                intake.setModeAngle(0.0)
            }
            else -> {
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(2.45)
            }
        }
        intake.intakePercentage = -1.0

    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }

}