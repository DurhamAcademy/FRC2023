package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ManipulatorConstants.manipulatorCurrent
import frc.robot.Constants.ManipulatorConstants.leftSolenoidForward
import frc.robot.Constants.ManipulatorConstants.leftSolenoidReverse
import frc.robot.Constants.ManipulatorConstants.motorId
import frc.robot.Constants.ManipulatorConstants.rightSolenoidForward
import frc.robot.Constants.ManipulatorConstants.rightSolenoidReverse

class Manipulator: SubsystemBase() {

    private val motor = CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        this.setSmartCurrentLimit(manipulatorCurrent)
    }
    private val leftsolenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, leftSolenoidForward, leftSolenoidReverse)
    private val rightsolenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, rightSolenoidForward, rightSolenoidReverse)

    var isOpen: Boolean?
        get() = leftsolenoid.get() == Value.kForward
        set(value) {
            val v =
                    if (value == null) Value.kOff
                    else if (value) Value.kForward
                    else Value.kReverse
            leftsolenoid.set(v)
            rightsolenoid.set(v)
        }

    var speed: Double
        get() = motor.get()
        set(value) {
            motor.set(value)
        }
}