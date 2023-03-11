package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.utils.PlacePoint
import frc.robot.RobotContainer
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator
import kotlin.math.sin

class SetPosition(
    val elevator: Elevator,
    val arm: Arm,
    val armPosition: Double,
    val elevatorPosition: Double,
    val wristPosition: Double,
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
    fun withManipulator(manipulator: Manipulator) =
        this.raceWith(CollectObject(manipulator))
    companion object {
        fun setpoint(placePoint: PlacePoint, elevator: Elevator, arm: Arm) =
            when (placePoint) {
                PlacePoint.Level1 -> low(elevator, arm)
                PlacePoint.Level2 -> mid(elevator, arm)
                PlacePoint.Level3 -> high(elevator, arm)
            }
        fun setpoint(placePoint: PlacePoint, robotContainer: RobotContainer) = setpoint(
            placePoint,
            robotContainer.elevator,
            robotContainer.arm
        )
        fun high(elevator: Elevator, arm: Arm) = SetPosition(
            elevator,
            arm,
            1.4,
            Constants.Elevator.limits.topLimit - inchesToMeters(2.0),
            Math.toRadians(-30.0)
        )
        fun mid(elevator: Elevator, arm: Arm) = SetPosition(
            elevator,
            arm,
            1.4,
            inchesToMeters(38.0),
            Math.toRadians(-30.0)
        )
        fun low(elevator: Elevator, arm: Arm) = SetPosition(
            elevator,
            arm,
            degreesToRadians(150.0),
            Constants.Elevator.limits.topLimit,
            Math.toRadians(-10.0)
        )
        fun humanPlayer(elevator: Elevator, arm: Arm) = SetPosition(
            elevator,
            arm,
            1.4,
            1.3- inchesToMeters(11.0),
            Math.toRadians(-5.0)
        )

        fun idle(robotContainer: RobotContainer): Command = SetPosition(
            robotContainer.elevator,
            robotContainer.arm,
            0.0,
            Constants.Elevator.limits.bottomLimit,
            Math.toRadians(0.0)
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
        val topLimit = Constants.FieldConstants.heightLimit
        val bottomLimit = 0.1
        val armLength = Constants.arm.length
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
    override fun isFinished(): Boolean = false // need to keep command running so wrist goes to right spot

}