package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import kotlin.math.sin

class SetSubsystemPosition (
    val elevator: Elevator,
    val arm: Arm,
    val level: IOLevel,

    val stopAtEnd: Boolean = true
) : CommandBase() {
    constructor(robotContainer: RobotContainer, level: IOLevel) : this(
        robotContainer.elevator,
        robotContainer.arm,
        level
    )

    init {
        addRequirements(arm)
        addRequirements(elevator)
    }
    override fun execute() {
        arm.setArmPosition(level.armRotation)
        val topLimit = FieldConstants.heightLimit
        val bottomLimit = 0.1
        val armLength = frc.robot.constants.arm.length
        // use arm angle to determine elevator height
        val armAngle = arm.armPosition
        val armHeight = armLength * sin(armAngle)
        val elevatorMaxHeight = topLimit - armHeight
        elevator.setpoint = level.elevatorHeight
            .coerceIn(bottomLimit, elevatorMaxHeight)
    }
}

