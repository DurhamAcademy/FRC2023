package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Manipulator

class IntakePositionForward(
    private val elevator: Elevator,
    private val arm: Arm,
    val stopAtEnd: Boolean = true
) : CommandBase() {

    init {
        addRequirements(elevator, arm)
    }

    override fun execute() {
        val armAngle = degreesToRadians(115.0)
//        wrist.setpoint
        arm.setArmPosition(armAngle)
        //Solver.getWristPose(0.0, armAngle, armAngle - PI / 2)
        elevator.setpoint = frc.robot.constants.elevator.limits.bottomLimit + inchesToMeters(1.0)
    }

    override fun isFinished(): Boolean {
        return stopAtEnd && elevator.height - (frc.robot.constants.elevator.limits.bottomLimit + inchesToMeters(1.0)) < 0.01 && arm.armPosition - degreesToRadians(
            125.0
        ) < 0.01
    }
}