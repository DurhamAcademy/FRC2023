package frc.robot.commands.intake

import frc.robot.RobotContainer
import frc.robot.subsystems.Intake
//import frc.robot.commands.intake.IntakeStates

class SetIntakeState(
    intake: Intake,
    state: IntakeStates
) : SetIntakePosition(
    intake,
    state.deployAngle,
    state.cubeArmAngle,
0.1//    state.intakePercentage
) {
    constructor(
        robotContainer: RobotContainer,
        state: IntakeStates
    ) : this(
        robotContainer.intake,
        state
    )
}