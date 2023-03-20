package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.utils.grid.PlacementLevel
import kotlin.math.absoluteValue
import kotlin.math.sin

class SetIntakePosition(
    val intake: Intake,
    val intakePosition: Double
) : CommandBase() {
    constructor(robotContainer: RobotContainer,
                intakePosition: Double) : this(
        robotContainer.intake,
        intakePosition
    )

    companion object {

        fun deploy(intake: Intake, position: Double) = SetPosition(

        )

        fun retract(elevator: Elevator, arm: Arm, stopAtEnd: Boolean = false) = SetPosition(
            elevator,
            arm,
            degreesToRadians(150.0),
            frc.robot.constants.elevator.limits.topLimit,
            Math.toRadians(-10.0),
            stopAtEnd
        )

    }
    init {
        addRequirements(intake)
    }

    override fun execute() {
        // change this to 48 inches
        //I think i did but double check this
        //arm.setArmPosition(armPosition)
        val topLimit = FieldConstants.heightLimit
        val bottomLimit = 0.1
        val armLength = frc.robot.constants.arm.length
        // use arm angle to determine elevator height
//        val armAngle = arm.armPosition
//        val armHeight = armLength * sin(armAngle)
        //val elevatorMaxHeight = topLimit - armHeight // if arm is at 0 degrees,
        // then the max height of the elevator is the top limit minus the length
        // of the arm

        // set elevator height
//        elevator.setpoint = elevatorPosition
//            .coerceIn(bottomLimit, elevatorMaxHeight)

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
    //override fun isFinished(): Boolean = stopAtEnd && (arm.armPosition - armPosition).absoluteValue < 0.1

}