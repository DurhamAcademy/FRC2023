package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Elevator
import frc.robot.utils.GamePiece
import kotlin.math.absoluteValue

class SetSubsystemPosition(
    val elevator: Elevator,
    val arm: Arm,
    val drivetrain: Drivetrain,
    inline val level: () -> IOLevel,
    inline val wantedObject: () -> GamePiece,
    val stopAtEnd: Boolean = false
) : CommandBase() {
    constructor(
        robotContainer: RobotContainer,
        level: () -> IOLevel,
        wantedObject: () -> GamePiece,
        stopAtEnd: Boolean = false
    ) : this(
        robotContainer.elevator,
        robotContainer.arm,
        robotContainer.drivetrain,
        level,
        wantedObject,
        stopAtEnd
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
        val ioLevel = level()
        if (ioLevel == IOLevel.StartingConfig) {
            arm.setArmPosition(ioLevel.cubeArmRotation.radians)
            return
        } else if (wantedObject() == GamePiece.cube) {
            goalArmPosition = ioLevel.cubeArmRotation.radians
            goalElevatorPosition = ioLevel.cubeElevatorHeight
        } else {
            goalArmPosition = ioLevel.coneArmRotation.radians
            goalElevatorPosition = ioLevel.coneElevatorHeight
        }

        arm.setArmPosition(goalArmPosition)
        // use arm angle to determine elevator height
//        val armAngle = arm.armPosition
//        val armHeight = armLength * sin(armAngle)
//        val elevatorMaxHeight = topLimit - armHeight

        SmartDashboard.putNumber("ELEVATOR GOAL", goalElevatorPosition)
        SmartDashboard.putNumber("cone elev", ioLevel.coneElevatorHeight)
        SmartDashboard.putString("Wanted", wantedObject().name)

        if (arm.armPosition > .2 && goalArmPosition < -.2 || arm.armPosition < -.2 && goalArmPosition > .2)
            elevator.setpoint = bottomLimit
        else
            elevator.setpoint = goalElevatorPosition
                .coerceAtLeast(bottomLimit)
    }

    override fun isFinished(): Boolean = stopAtEnd &&
            (arm.armPosition - goalArmPosition).absoluteValue < 0.1 &&
            (elevator.height - goalElevatorPosition).absoluteValue < 0.05
}



