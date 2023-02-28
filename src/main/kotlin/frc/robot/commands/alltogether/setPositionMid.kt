package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Arm
import frc.robot.controls.ControlScheme

class setPositionMid(
    val elevator: Elevator,
    val arm: Arm,
    val controlScheme: ControlScheme,
): CommandBase() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
    }

    override fun execute() {
        // change this to 34 inches(idk how)
        //I think i did but double check this
        elevator.setpoint = (Constants.Elevator.limits.bottomLimit + 34)

        arm.setArmPosition(1.4)
    }

    //double check the arm position thing
    override fun isFinished(): Boolean =
        elevator.height == 34.0 && arm.armPosition == 1.4

}