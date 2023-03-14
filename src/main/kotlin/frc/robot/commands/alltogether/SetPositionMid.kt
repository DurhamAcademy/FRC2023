package frc.robot.commands.alltogether

import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator

class SetPositionMid(
    val elevator: Elevator,
    val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(arm)
        addRequirements(elevator)
    }

    override fun execute() {
        // change this to 34 inches(idk how)
        //I think i did but double check this
        arm.setArmPosition(1.4)

        if (arm.armPosition < 0.25) {
            elevator.setpoint = frc.robot.constants.elevator.limits.bottomLimit
        } else {
            elevator.setpoint = inchesToMeters(34.0)
        }

//        if (elevator.height < inchesToMeters(30.0)) {
//            wrist.setPosition(wrist.levelAngle(Math.toRadians(60.0)))
//        } else {
//            wrist.setPosition(-.26)
//        }
    }

    //double check the arm position thing
    override fun isFinished(): Boolean =
        elevator.motorPid.atGoal()
                && arm.armPID.atGoal()
//                && wrist.pid.atGoal()

}