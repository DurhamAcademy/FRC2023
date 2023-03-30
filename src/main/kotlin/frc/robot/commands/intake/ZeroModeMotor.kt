package frc.robot.commands.intake

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Intake

class ZeroModeMotor(private val intake: Intake): CommandBase() {

    val timer = Timer()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        timer.restart()
        intake.modeZeroed = false
        intake.setDeployAngle(intake.deployPosition)
    }
    override fun execute() {
        intake.modeVoltage = -3.0
    }

    override fun end(interrupted: Boolean) {
        intake.zeroModeMotor()
        intake.modeVoltage = 0.0
    }

    override fun isFinished() = timer.hasElapsed(1.0)

}