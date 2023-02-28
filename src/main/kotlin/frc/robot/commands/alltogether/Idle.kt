package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist

class Idle(
    private val elevator: Elevator,
    private val arm: Arm,
    private val wrist: Wrist,
) : CommandBase() {
    init {
        addRequirements(elevator, arm, wrist)
    }

    override fun execute() {
        elevator.setpoint = Constants.Elevator.limits.bottomLimit
        arm.setArmPosition(0.0)
        wrist.setPosition(wrist.levelAngle(50.0))
    }

    override fun isFinished(): Boolean =
        elevator.motorPid.atGoal()
                && arm.armPID.atGoal()
                && wrist.pid.atGoal()
}