package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator
import frc.robot.utils.grid.PlacmentLevel
import kotlin.math.absoluteValue
import kotlin.math.sin

class SetPosition(
    val elevator: Elevator,
    val arm: Arm,
    val armPosition: Double,
    val elevatorPosition: Double,
    val wristPosition: Double,
    val stopAtEnd: Boolean = true,
) : CommandBase() {
    constructor(robotContainer: RobotContainer,
                armPosition: Double,
                elevatorPosition: Double,
                wristPosition: Double) : this(
        robotContainer.elevator,
        robotContainer.arm,
        armPosition,
        elevatorPosition,
        wristPosition
    )

    companion object {
        fun setpoint(placmentLevel: PlacmentLevel, elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) =
            when (placmentLevel) {
                PlacmentLevel.Level1 -> low(elevator, arm, stopAtEnd)
                PlacmentLevel.Level2 -> mid(elevator, arm, stopAtEnd)
                PlacmentLevel.Level3 -> high(elevator, arm, stopAtEnd)
            }

        fun setpoint(placmentLevel: PlacmentLevel, robotContainer: RobotContainer, stopAtEnd: Boolean = false) = setpoint(
            placmentLevel,
            robotContainer.elevator,
            robotContainer.arm,
            stopAtEnd
        )

        fun high(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) = SetPosition(
            elevator,
            arm,
            1.4,
            frc.robot.constants.elevator.limits.topLimit - inchesToMeters(2.0),
            Math.toRadians(-30.0),
            stopAtEnd
        )

        fun mid(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) = SetPosition(
            elevator,
            arm,
            1.4,
            inchesToMeters(38.0),
            Math.toRadians(-30.0),
            stopAtEnd
        )

        fun low(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) = SetPosition(
            elevator,
            arm,
            degreesToRadians(150.0),
            frc.robot.constants.elevator.limits.topLimit,
            Math.toRadians(-10.0),
            stopAtEnd
        )

        fun humanPlayer(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) = SetPosition(
            elevator,
            arm,
            1.4,
            1.3 - inchesToMeters(11.0),
            Math.toRadians(-5.0),
            stopAtEnd
        )

        fun idle(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false): Command = SetPosition(
            elevator,
            arm,
            0.0,
            frc.robot.constants.elevator.limits.bottomLimit,
            Math.toRadians(0.0),
            stopAtEnd
        )
    }
    init {
        addRequirements(arm)
        addRequirements(elevator)
    }

    override fun execute() {
        // change this to 48 inches
        //I think i did but double check this
        arm.setArmPosition(armPosition)
        val topLimit = FieldConstants.heightLimit
        val bottomLimit = 0.1
        val armLength = frc.robot.constants.arm.length
        // use arm angle to determine elevator height
        val armAngle = arm.armPosition
        val armHeight = armLength * sin(armAngle)
        val elevatorMaxHeight = topLimit - armHeight // if arm is at 0 degrees,
        // then the max height of the elevator is the top limit minus the length
        // of the arm

        // set elevator height
        elevator.setpoint = elevatorPosition
            .coerceIn(bottomLimit, elevatorMaxHeight)

        // set wrist angle if we are approaching the final position of the
        // elevator and arm
//        if (
//            (elevator.height - elevatorPosition).absoluteValue < 6.0
//            && (arm.armPosition - armPosition).absoluteValue < 1.0
//            ) {
//            wrist.setPosition(wrist.levelAngle(Math.toRadians(30.0)) + wristPosition)
//        } else {
//            wrist.setPosition(wrist.levelAngle(Math.toRadians(50.0)))
//        }
    }

    //double-check the arm position thing
    override fun isFinished(): Boolean = stopAtEnd && (arm.armPosition - armPosition).absoluteValue < 0.1

}