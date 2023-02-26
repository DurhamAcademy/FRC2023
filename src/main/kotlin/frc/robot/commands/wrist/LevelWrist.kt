package frc.robot.commands.wrist

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Wrist

class LevelWrist(
    private val wrist: Wrist,
    private val arm: Arm,
    val angleOfFlip : Double = 20.0,
) : CommandBase() {
    init {
        addRequirements(wrist)
    }
    fun calcAngles(): Double {
        if(arm.armPosition < -angleOfFlip){
            return -(90 + arm.armPosition)
            /*
               If arm is in negative direction set wrist level to floor
               The arm and wrist must add up to 90 (e.g. if the arm is flat at 90 degrees the wrist must be at 0 degrees)
               As armPosition is negative 90 + armPosition is the remaining amount that adds up to 90
               Then it is multiplied by -1 to keep it on the same side
             */
        }
        else if(arm.armPosition > angleOfFlip){
            return 90 - arm.armPosition
            /*
               If arm is in the positive direction set wrist level to floor
               90 - armPosition is the amount wrist needs to be set at
             */
        }
        else if(arm.armPosition > 0){
            return arm.armPosition * ((90-angleOfFlip)/angleOfFlip) //if arm is in the middle flip wrist
        }
        else{
            return arm.armPosition * -((90-angleOfFlip)/angleOfFlip) //if arm is in the middle flip wrist
        }
    }
    override fun execute() {
        wrist.setPosition(Math.toRadians(calcAngles()))
    }

    override fun isFinished() = false
}