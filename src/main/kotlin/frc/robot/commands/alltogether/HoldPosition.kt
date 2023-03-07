package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator

class HoldPosition(
    val elevator: Elevator,
    val arm: Arm,
) : InstantCommand() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
    }

    override fun initialize() {
        elevator.setpoint = elevator.height
        arm.setArmPosition(arm.armPosition)
    }
}