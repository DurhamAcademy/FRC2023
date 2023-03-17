package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator

class ZeroElevatorAndIdle(
    val elevator: Elevator,
    val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(elevator)
        addRequirements(arm)
    }

    override fun initialize() {
        println("Zeroing Elevator")
    }

    override fun execute() {
        elevator.zeroElevator = true
        arm.setArmPosition(0.0)
    }

    override fun end(interrupted: Boolean) {
        elevator.zeroElevator = false
        println("Elevator Zeroed")
    }

    override fun isFinished(): Boolean = elevator.limitSwitchPressed
}