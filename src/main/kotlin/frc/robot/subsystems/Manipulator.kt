package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ManipulatorConstants.leftSolenoidForward
import frc.robot.Constants.ManipulatorConstants.leftSolenoidReverse
import frc.robot.Constants.ManipulatorConstants.manipulatorVoltage
import frc.robot.Constants.ManipulatorConstants.motorId
import frc.robot.Constants.ManipulatorConstants.rightSolenoidForward
import frc.robot.Constants.ManipulatorConstants.rightSolenoidReverse


class Manipulator: SubsystemBase() {

    private val motor = CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        this.setSmartCurrentLimit(manipulatorVoltage)
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


    val i2cPort = I2C.Port.kOnboard
    val colorSensor = ColorSensorV3(i2cPort)


    var proximity: Double
        get() = colorSensor.proximity.toDouble()
        set(value) {
        }

    var color: Color
        get() = colorSensor.color
        set(value) {
        }

    fun getDist(): Double {
        return proximity
    }

    fun getBlue(): Double {
        return color.blue
    }

    fun getRed(): Double {
        return color.red
    }
    fun getGreen(): Double {
        return color.green
    }


    override fun periodic() {
        val detectedColor: Color = colorSensor.color
        val IR = colorSensor.ir.toDouble()

        SmartDashboard.putNumber("Red", detectedColor.red)
        SmartDashboard.putNumber("Green", detectedColor.green)
        SmartDashboard.putNumber("Blue", detectedColor.blue)
        SmartDashboard.putNumber("IR", IR)

        val proximity = colorSensor.proximity
        SmartDashboard.putNumber("Proximity", proximity.toDouble())
    }
}