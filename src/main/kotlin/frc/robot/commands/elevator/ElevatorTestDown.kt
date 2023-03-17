package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Elevator

// go up until the top limit is reached then end
class ElevatorTestDown(
    var elevator: Elevator,
) : CommandBase() {
    init {
        addRequirements(elevator)
    }

    override fun initialize() {
        elevator.setpoint = frc.robot.constants.elevator.limits.bottomLimit
    }

    override fun execute() {
        elevator.setpoint = frc.robot.constants.elevator.limits.bottomLimit
    }

    override fun isFinished(): Boolean {
        return elevator.height < frc.robot.constants.elevator.limits.bottomLimit + 0.1
    }
}