package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.controls.ControlScheme
import java.util.ResourceBundle.Control


class Intake(): SubsystemBase() {

    private val pid = ProfiledPIDController(
            Constants.intake.motor.kP,
            Constants.intake.motor.kI,
            Constants.intake.motor.kD,
            TrapezoidProfile.Constraints(
                    Constants.intake.motor.maxVelocity,
                    Constants.intake.motor.maxAcceleration
            )
    ).apply {
        setTolerance(
                Constants.intake.motor.positionTolerance,
                Constants.intake.motor.velocityTolerance
        )
    }

    private val spinningMotor = CANSparkMax(
            Constants.intakeSpinMotorID,
            CANSparkMaxLowLevel.MotorType.kBrushless
    )

    private val deployMotor = CANSparkMax(
            Constants.intakeDeployMotorID,
            CANSparkMaxLowLevel.MotorType.kBrushless
    )

    var intakeSetpoint: Double? = null
    var setpoint: Double? = null

    var spinMotorSpeed: Double
        get() = spinningMotor.get()
        set(value) {
            spinningMotor.set(value)
        }

    var voltage: Double = 0.0
        set(value) {
            deployMotor.setVoltage(voltage)
            field = value
        }

    /**
     * spinIntakeMotor function
     * spins the motor with the compliant wheels on intake
     * @param speed, a double that represents the speed that the intake motor is set to
     */
    fun spinIntakeMotor(speed: Double) {
        spinMotorSpeed = speed
    }

    fun setIntakeDeployVoltage(voltage: Double) {
        deployMotor.setVoltage(voltage)
    }

    /**
     * setIntakePosition function
     * @param position, a double from 0.0 to 1.0 that represents the percentage that the intake is deployed
     * for example, 0.5 would mean 50% deployed
     */
    fun setIntakePosition(position: Double) {
        intakeSetpoint = position
    }




    override fun periodic() {
        if (setpoint != null) {
            val output = pid.calculate(setpoint!!)
            voltage = output
        } else voltage = 0.0
    }
}
