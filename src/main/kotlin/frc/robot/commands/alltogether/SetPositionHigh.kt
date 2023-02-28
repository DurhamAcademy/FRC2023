package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Wrist

class SetPositionHigh(
    val elevator: Elevator,
    val arm: Arm,
    val wrist: Wrist
) : CommandBase() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
        addRequirements(wrist)
    }

    override fun execute() {
        // change this to 48 inches(idk how)
        //I think i did but double check this
        arm.setArmPosition(1.4)

        if (arm.armPosition < 0.25) {
            elevator.setpoint = Constants.Elevator.limits.bottomLimit
        } else {
            elevator.setpoint = (Constants.Elevator.limits.bottomLimit + 48)
        }

        if (elevator.height < 25) {
            wrist.setPosition(wrist.levelAngle(20.0))
        } else {
            wrist.setPosition(wrist.levelAngle(60.0))
        }
    }

    //double check the arm position thing
    override fun isFinished(): Boolean =
        elevator.motorPid.atGoal()
                && arm.armPID.atGoal()
                && wrist.pid.atGoal()

}