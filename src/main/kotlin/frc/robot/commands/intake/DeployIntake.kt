package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.kyberlib.command.Game
import frc.robot.RobotContainer
import frc.robot.constants.intake
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece

class DeployIntake(
    intake: Intake,
    states: IntakeStates,
    intakePercentage: Double,
    val robotContainer: RobotContainer
) : SetIntakePosition(
    intake,
    states.deployAngle,
    states.cubeArmAngle,
    intakePercentage
) {

    fun init(){
        addRequirements(intake)
    }

    override fun execute(){
        intake.setDeployAngle(deployAngle)
        when (robotContainer.wantedObject) {
            GamePiece.cone -> {
                intake.setModeAngle(0.0)
            }
            GamePiece.cube -> {
                intake.setModeAngle(2.88)
            }
            else -> {
                intake.setModeAngle(0.0)
            }
        }
        intake.intakePercentage = 0.5

    }

    override fun end(interrupted: Boolean){

    }

}