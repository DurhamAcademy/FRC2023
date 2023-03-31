package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.manipulator.SetManipulatorSpeed
import frc.robot.constants.manipulator.motorId


class Manipulator : SubsystemBase() {

    private val motor = CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        // make the motor report less often to reduce network traffic
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20)
    }

    var motorPercentage: Double
        get() = motor.get()
        set(value) {
            motor.set(value)
        }

    private val tab = Shuffleboard.getTab("Manipulator")
    private val motorCurrent = tab.add("Motor Current", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry

    init {
        defaultCommand = SetManipulatorSpeed(this, 0.05)
    }

    override fun periodic() {
        motorCurrent.setDouble(motor.outputCurrent)
    }
}