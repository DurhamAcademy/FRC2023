package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Arm
import frc.robot.controls.ControlScheme

class setPositionHigh(
    val elevator: Elevator,
    val arm: Arm,
    val controlScheme: ControlScheme,
): CommandBase() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
    }

    override fun execute() {
        // change this to 48 inches(idk how)
        elevator.setpoint = (Constants.Elevator.limits.topLimit -
                Constants.Elevator.limits.bottomLimit) / 2 +
                Constants.Elevator.limits.bottomLimit

        arm.setArmPosition(1.4)
    }

    //double check the arm position thing
    override fun isFinished(): Boolean =
        elevator.height > Constants.Elevator.limits.topLimit - 0.1 && arm.armPosition == 1.4

}