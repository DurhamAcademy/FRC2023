package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator

class Idle(
    private val elevator: Elevator,
    private val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(elevator, arm)
    }

    override fun execute() {
        elevator.setpoint = frc.robot.constants.elevator.limits.bottomLimit
        arm.setArmPosition(0.0)
    }

    override fun isFinished(): Boolean =
        elevator.motorPid.atGoal()
                && arm.armPID.atGoal()
}