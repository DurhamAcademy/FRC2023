package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Manipulator: SubsystemBase() {

    private val motor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        this.setSmartCurrentLimit(20)
    }
    private val solenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1)
    private val solenoid2 = DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3)

    var open: Boolean
        get() = solenoid.get() == DoubleSolenoid.Value.kForward
        set(value) {
            val v = when(value) {
                true -> DoubleSolenoid.Value.kForward
                false -> DoubleSolenoid.Value.kReverse
            }
            solenoid.set(v)
            solenoid2.set(v)
        }

    var speed: Double
        get() = motor.get()
        set(value) {
            motor.set(value)
        }
}