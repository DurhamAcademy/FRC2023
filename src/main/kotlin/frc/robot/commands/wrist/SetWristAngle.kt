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
    }

    override fun initialize() {
    }

    override fun execute() {
        if(arm.armPosition < -20){
            wrist.setPosition((90 + arm.armPosition) * -1)
            /*
               If arm is in negative direction set wrist level to floor
               The arm and wrist must add up to 90 (e.g. if the arm is flat at 90 degrees the wrist must be at 0 degrees)
               As armPosition is negative 90 + armPosition is the remaining amount that adds up to 90
               Then it is multiplied by -1 to keep it on the same side
             */
        }
        else if(arm.armPosition > 20){
            wrist.setPosition(90 - arm.armPosition)
            /*
               If arm is in the positive direction set wrist level to floor
               90 - armPosition is the amount wrist needs to be set at
             */
        }
        /*
          The wrist needs to go from level with floor on one side to level with floor on the other side (70 degrees to -70 degrees) aka 140 degrees over 40 degrees
          140/40 = 3.5
          If arm is positive then the wrist position starts at 3.5 * 20 (armPosition) = 70
          If arm is positive then the wrist position starts at -3.5 * 20 (armPosition) = -70
          Both of these then get multiplied by less and less as the arm goes up and get closer to the 0 degree point
         */
        else if(arm.armPosition > 0){
            wrist.setPosition(arm.armPosition * 3.5) //if arm is in the middle flip wrist
        }
        else{
            wrist.setPosition(arm.armPosition * -3.5) //if arm is in the middle flip wrist
        }
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean {
        //don't know what to do here
        return true
    }
}