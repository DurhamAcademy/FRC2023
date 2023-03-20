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
    inline val level: () -> IOLevel,
    inline val wantedObject: () -> GamePiece,

    val stopAtEnd: Boolean = false
) : CommandBase() {
    constructor(robotContainer: RobotContainer, level: () -> IOLevel, wantedObject: () -> GamePiece) : this(
        robotContainer.elevator,
        robotContainer.arm,
        level,
        wantedObject,
    )
    val topLimit = FieldConstants.heightLimit
    val bottomLimit = 0.1
    val armLength = frc.robot.constants.arm.length
    var goalArmPosition = 0.0
    var goalElevatorPosition = 0.0

    init {
        addRequirements(arm)
        addRequirements(elevator)
    }
    override fun execute() {
        if(level() == IOLevel.StartingConfig){
            goalArmPosition = level().cubeArmRotation.radians
            goalElevatorPosition = elevator.height
        }
        else if(wantedObject() == GamePiece.cube){
            goalArmPosition = level().cubeArmRotation.radians
            goalElevatorPosition = level().cubeElevatorHeight
        }
        else{
            goalArmPosition = level().coneArmRotation.radians
            goalElevatorPosition = level().coneElevatorHeight
        }
        arm.setArmPosition(goalArmPosition)
        // use arm angle to determine elevator height
        val armAngle = arm.armPosition
        val armHeight = armLength * sin(armAngle)
        val elevatorMaxHeight = topLimit - armHeight
        if(arm.armPosition > .2 && goalArmPosition < -.2 || arm.armPosition < -.2 && goalArmPosition > .2)
            elevator.setpoint = bottomLimit
        else
            elevator.setpoint = goalElevatorPosition
                .coerceIn(bottomLimit, elevatorMaxHeight)
    }

    override fun isFinished(): Boolean = stopAtEnd &&
            (arm.armPosition - goalArmPosition).absoluteValue < 0.1 &&
            (elevator.height - goalElevatorPosition).absoluteValue < 0.05
}

