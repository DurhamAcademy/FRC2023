package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.utils.GamePiece
import kotlin.math.absoluteValue
import kotlin.math.sin

class SetSubsystemPosition (
    val elevator: Elevator,
    val arm: Arm,
    val level: IOLevel,
    val wantedObject: GamePiece,

    val stopAtEnd: Boolean = true
) : CommandBase() {
    constructor(robotContainer: RobotContainer, level: IOLevel, wantedObject: GamePiece) : this(
        robotContainer.elevator,
        robotContainer.arm,
        level,
        wantedObject,
    )
    val topLimit = FieldConstants.heightLimit
    val bottomLimit = 0.1
    val armLength = frc.robot.constants.arm.length
    var goalArmPosition = 0.0

    init {
        addRequirements(arm)
        addRequirements(elevator)
    }
    override fun execute() {
        if(wantedObject == GamePiece.cube){
            arm.setArmPosition(level.cubeArmRotation)
            goalArmPosition = level.cubeArmRotation.degrees
            // use arm angle to determine elevator height
            val armAngle = arm.armPosition
            val armHeight = armLength * sin(armAngle)
            val elevatorMaxHeight = topLimit - armHeight
            elevator.setpoint = level.cubeElevatorHeight
                .coerceIn(bottomLimit, elevatorMaxHeight)
        }
        else{
            arm.setArmPosition(level.coneArmRotation)
            goalArmPosition = level.coneArmRotation.degrees
            // use arm angle to determine elevator height
            val armAngle = arm.armPosition
            val armHeight = armLength * sin(armAngle)
            val elevatorMaxHeight = topLimit - armHeight
            elevator.setpoint = level.coneElevatorHeight
                .coerceIn(bottomLimit, elevatorMaxHeight)
        }
    }
    override fun isFinished(): Boolean = stopAtEnd && (arm.armPosition - goalArmPosition).absoluteValue < 0.1

}

