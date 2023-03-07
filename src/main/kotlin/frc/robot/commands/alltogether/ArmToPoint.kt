package frc.robot.commands.alltogether

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Arm
import frc.robot.subsystems.Elevator
import kotlin.math.*

class ArmToPoint (
    var elevator: Elevator,
    var arm: Arm,
) : CommandBase() {
    init {
        addRequirements(elevator)
        addRequirements(arm)
    }
    /**
     * Input (x,y,armLength) cords of where arm should end up
     * Set elevator to y + sqrt(arm length^2 - x^2)
     * Set arm to inverse sin of x/arm length or PI - (inverse sin of x/arm length)
     */
    fun moveArmForPoint(x : Double, y: Double, armLength: Double){
        if(elevator.height - abs(y + sqrt(armLength.pow(2) - x.pow(2))) < elevator.height - abs(y - sqrt(armLength.pow(2) - x.pow(2)))){
            elevator.setpoint = y + sqrt(armLength.pow(2) - x.pow(2))
            arm.setArmPosition(PI - asin(x/armLength))
        }
        else {
            elevator.setpoint = y - sqrt(armLength.pow(2) - x.pow(2))
            arm.setArmPosition(asin(x/armLength))
        }
    }
}