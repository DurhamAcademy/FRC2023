package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Elevator

// go up until the top limit is reached then end
class ElevatorTestUp(
    var elevator: Elevator,
) : CommandBase() {
    init {
        addRequirements(elevator)
    }

    override fun initialize() = Unit

    override fun execute() {
        elevator.setpoint = (Constants.Elevator.limits.topLimit -
                Constants.Elevator.limits.bottomLimit) / 2 +
                Constants.Elevator.limits.bottomLimit
    }

    override fun isFinished(): Boolean =
        elevator.height > Constants.Elevator.limits.topLimit - 0.1
}