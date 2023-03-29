package frc.robot.commands.intake

import frc.robot.subsystems.Intake
import frc.robot.utils.GamePiece

class DeployIntake(
    intake: Intake,
    gamePiece: GamePiece,
    intakePercentage: Double
) : SetIntakePosition(
    intake,
    gamePiece.deployAngle,
    gamePiece.cubeArmAngle,
    intakePercentage
) {
    constructor(
        intake: Intake,
        gamePiece: GamePiece
    ) : this(
        intake,
        gamePiece,
        0.0
    )
}