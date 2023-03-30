package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece
import kotlin.math.PI

class DeployIntake(
    val intake: Intake,
    val robotContainer: RobotContainer,
    val gamePiece: () -> GamePiece,
): CommandBase() {

    init{
        addRequirements(intake)
    }

    override fun execute(){
        when (gamePiece()) {
            //SWAPPED
            GamePiece.cube -> {
                intake.modeVoltage = -3.0
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(1.97)
            }

            GamePiece.cone -> {
                intake.modeVoltage = -3.0
                intake.setDeployAngle(PI * 2 - 1.4)
                intake.setModeAngle(0.1)
            }

            else -> {
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(1.97)
            }
        }
        intake.intakePercentage = -1.0

    }

    override fun end(interrupted: Boolean){
        intake.intakePercentage = 0.0
    }

}