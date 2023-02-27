package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist
import kotlin.math.absoluteValue

class P2(
    val arm: Arm,
    val elevator: Elevator,
    val wrist: Wrist,
) : CommandBase() {
    init {
        addRequirements(arm, elevator, wrist)
    }


    override fun execute() {
        arm.armSetpoint = 1.4
        wrist.setpoint = 0.0
        if (arm.armPosition < 0.1) {
            elevator.setpoint = 0.0
        } else {
            elevator.setpoint = 45.0
        }
    }

    override fun isFinished(): Boolean =
        (arm.armPosition - arm.armPID.goal.position).absoluteValue < 0.1
                && (wrist.position - wrist.pid.goal.position).absoluteValue < 0.1
                && (elevator.height - 45).absoluteValue < 5
}