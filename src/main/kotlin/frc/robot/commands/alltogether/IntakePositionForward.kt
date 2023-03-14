package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator

class IntakePositionForward(
    private val elevator: Elevator,
    private val arm: Arm,
    val stopAtEnd: Boolean = true
) : CommandBase() {
    fun withManipulator(manipulator: Manipulator) =
        this
            .raceWith(CollectObject(manipulator))

    init {
        addRequirements(elevator, arm)
    }

    override fun execute() {
        val armAngle = degreesToRadians(125.0)
//        wrist.setpoint
        arm.setArmPosition(armAngle)
        //Solver.getWristPose(0.0, armAngle, armAngle - PI / 2)
        elevator.setpoint = inchesToMeters(7.0 + 20 + 1 + 2)
    }

    override fun isFinished(): Boolean {
        return stopAtEnd && elevator.height - inchesToMeters(7.0 + 20 + 1 + 2) < 0.01 && arm.armPosition - degreesToRadians(
            125.0
        ) < 0.01
    }
}