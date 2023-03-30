package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece
import kotlin.math.PI

class IdleIntake (
    private val intake: Intake,
) : CommandBase() {

    init{
        addRequirements(intake)
    }

    override fun execute() {
        when (GamePiece.cone) {
            //SWAPPED
            GamePiece.cone -> {
                intake.intakePercentage = -0.1
                intake.modeVoltage = -3.0
                intake.setDeployAngle(0.3)
                intake.setModeAngle(2.0)

            }
            GamePiece.cube -> {
                intake.intakePercentage = -0.1
                intake.modeVoltage = -3.0
                intake.setDeployAngle(0.3)
                intake.setModeAngle(0.0)
            }
            else -> {
                intake.setDeployAngle(PI * 2 - 1.047)
                intake.setModeAngle(2.45)
            }
        }
    }

    override fun isFinished() = false
}

