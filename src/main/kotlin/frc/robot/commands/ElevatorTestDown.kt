package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Elevator

class ElevatorTestDown(
    var elevator: Elevator,
    var controlScheme: ControlScheme,
) : CommandBase() {
    init {
        addRequirements(elevator)
    }

    override fun execute() {
        elevator.setpoint = Constants.Elevator.limits.bottomLimit
    }

    override fun isFinished(): Boolean {
        return elevator.height <= Constants.Elevator.limits.bottomLimit + 0.1
    }
}