package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist

class HoldPosition(
    val elevator: Elevator,
    val arm: Arm,
    val wrist: Wrist
) : InstantCommand() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
        addRequirements(wrist)
    }

    override fun initialize() {
        elevator.setpoint = elevator.height
        arm.setArmPosition(arm.armPosition)
        wrist.setPosition(wrist.position)
    }
}