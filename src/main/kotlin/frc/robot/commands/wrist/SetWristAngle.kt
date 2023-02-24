package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Wrist

class SetWristAngle(
    private val wrist: Wrist,
    private val arm: Arm,
) : CommandBase() {
    init {
        addRequirements(wrist)
        addRequirements(arm)
    }

    override fun initialize() {
        wrist.setPosition(90.0)
    }

    override fun execute() {
        if(arm.armPosition < -20){
            wrist.setPosition((90 + arm.armPosition) * -1)
        }
        else if(arm.armPosition > 20){
            wrist.setPosition(90 - arm.armPosition)
        }
        else if(arm.armPosition > 0){
            wrist.setPosition(arm.armPosition * -3.5)
        }
        else{
            wrist.setPosition(arm.armPosition * 3.5)
        }
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        return true
    }
}