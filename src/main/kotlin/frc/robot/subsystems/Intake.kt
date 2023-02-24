package frc.robot.subsystems
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase

import frc.robot.controls.ControlScheme


class Intake(

): SubsystemBase() {
    private val deployMotor = CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val spinningMotor = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
    var spinMotorSpeed: Double
        get() = spinningMotor.get()
        set(value) {
            spinningMotor.set(value)
        }
    var deployMotorSpeed: Double
        get() = deployMotor.get()
        set(value) {
            deployMotor.set(value)
        }


}
