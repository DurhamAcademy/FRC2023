package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.CommandBase
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
        elevator.setpoint = (frc.robot.constants.elevator.limits.topLimit -
                frc.robot.constants.elevator.limits.bottomLimit) / 2 +
                frc.robot.constants.elevator.limits.bottomLimit
    }

    override fun isFinished(): Boolean =
        elevator.height > frc.robot.constants.elevator.limits.topLimit - 0.1
}