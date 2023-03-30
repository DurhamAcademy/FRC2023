package frc.robot.commands.intake

import frc.robot.subsystems.Intake

class IdleIntake (
    intake: Intake,
    states: IntakeStates,
    intakePercentage: Double
) : SetIntakePosition(
    intake,
    states.deployAngle,
    states.cubeArmAngle,
    intakePercentage
) {
    constructor(
        intake: Intake,
        states: IntakeStates
    ) : this(
        intake,
        states,
        0.0
    )

    fun init(){
        addRequirements(intake)
    }

    override fun initialize() {
        intake.setDeployAngle(0.0)
        intake.setModeAngle(0.0)
    }

    override fun isFinished() =
        intake.deployPID.atGoal() &&
        intake.modePID.atGoal()
}

