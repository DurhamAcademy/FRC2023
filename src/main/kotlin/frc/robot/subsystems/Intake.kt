package frc.robot.subsystems

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.arm
import frc.robot.constants.elevator
import frc.robot.constants.intake

class Intake : SubsystemBase()  {
    private val driveMotor = CANSparkMax(intake.driveMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        setSmartCurrentLimit(intake.driveMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    private val modeMotor = CANSparkMax(intake.modeMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        setSmartCurrentLimit(intake.modeMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    private val systemMotor = CANSparkMax(intake.systemMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        setSmartCurrentLimit(intake.systemMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }

    val systemEncoder = CANCoder(intake.systemencoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(intake.systemencoder.offset)
        configSensorDirection(intake.systemencoder.inverted)
    }

    val modeEncoder = modeMotor.getEncoder()

    var driveMotorPercentage: Double
        get() = driveMotor.get()
        set(value) {
            driveMotor.set(value)
        }
    var modeMotorPercentage: Double
        get() = modeMotor.get()
        set(value) {
            modeMotor.set(value)
        }
    var systemMotorPercentage: Double
        get() = systemMotor.get()
        set(value) {
            systemMotor.set(value)
        }
    val limitSwitch = DigitalInput(
        intake.limitSwitch.intakeLimitSwitch
    )

    val intakeSystemPID = ProfiledPIDController(
        intake.systemmotor.kP,
        intake.systemmotor.kI,
        intake.systemmotor.kD,
        TrapezoidProfile.Constraints(
            intake.systemmotor.maxVelocity,
            intake.systemmotor.maxAcceleration
        )
    ).apply {
        setTolerance(
            intake.systemmotor.positionTolerance,
            intake.systemmotor.velocityTolerance
        )
    }

    val limitSwitchPressed: Boolean
        get() = !limitSwitch.get()

    val modeMotorPosition: Double
        get() = Math.toRadians(modeEncoder.position)

    val systemMotorPosition: Double
        get() = Math.toRadians(systemEncoder.absolutePosition)

    val tab = Shuffleboard.getTab("Manipulator")
    val driveMotorCurrent = tab.add("Motor Current", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry
    val modeMotorCurrent = tab.add("Motor Current", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry
    val systemMotorCurrent = tab.add("Motor Current", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry

    fun getVoltage(): Double {
        return modeMotor.busVoltage
    }

    fun setIntakePosition(position: Double) {
        intakeSetpoint = position.coerceIn(
            arm.minAngle,
            arm.maxAngle
        )
    }

    override fun periodic() {
        driveMotorCurrent.setDouble(driveMotor.outputCurrent)
        modeMotorCurrent.setDouble(modeMotor.outputCurrent)
        systemMotorCurrent.setDouble(systemMotor.outputCurrent)
    }
}