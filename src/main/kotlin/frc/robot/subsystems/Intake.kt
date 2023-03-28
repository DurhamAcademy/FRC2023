package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.intake

class Intake : SubsystemBase()  {
    private val driveMotor = CANSparkMax(intake.driveMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        setSmartCurrentLimit(intake.driveMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    private val modeMotor = CANSparkMax(intake.modeMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        setSmartCurrentLimit(intake.modeMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    private val systemMotor = CANSparkMax(
        intake.systemMotorId,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        setSmartCurrentLimit(intake.systemMotorLimit) // add current limit to limit the torque
        idleMode = CANSparkMax.IdleMode.kBrake
    }

    val systemEncoder = AnalogEncoder(intake.baseEncoder.id).apply {
        //@TODO config analog bore encoder
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

    val systemPID = ProfiledPIDController(
        intake.baseMotor.kP,
        intake.baseMotor.kI,
        intake.baseMotor.kD,
        TrapezoidProfile.Constraints(
            intake.baseMotor.maxVelocity,
            intake.baseMotor.maxAcceleration
        )
    ).apply {
        setTolerance(
            intake.baseMotor.positionTolerance,
            intake.baseMotor.velocityTolerance
        )
    }

    val modePID = ProfiledPIDController(
        intake.baseMotor.kP,
        intake.baseMotor.kI,
        intake.baseMotor.kD,
        TrapezoidProfile.Constraints(
            intake.baseMotor.maxVelocity,
            intake.baseMotor.maxAcceleration
        )
    ).apply {
        setTolerance(
            intake.baseMotor.positionTolerance,
            intake.baseMotor.velocityTolerance
        )
    }

    val limitSwitchPressed: Boolean
        get() = !limitSwitch.get()

    val modeMotorPosition: Double
        get() = Math.toRadians(modeEncoder.position)
    //@TODO Setter

    val systemMotorPosition: Double
        get() = Math.toRadians(systemEncoder.absolutePosition)
    //@TODO Setter

    val tab = Shuffleboard.getTab("Intake")
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

    private var intakeSetpoint: Double? = null

    fun setIntakePosition(position: Double) {
        intakeSetpoint = position.coerceIn(
            intake.minAngle,
            intake.maxAngle
        )
    }

    val intakePosition: Double
        get() = if (RobotBase.isSimulation()) systemPID.setpoint.position//simArmSystem.angleRads
        else Math.toRadians(systemEncoder.absolutePosition)

    override fun periodic() {
        driveMotorCurrent.setDouble(driveMotor.outputCurrent)
        modeMotorCurrent.setDouble(modeMotor.outputCurrent)
        systemMotorCurrent.setDouble(systemMotor.outputCurrent)

        val calculate = systemPID.calculate(
            intakePosition,
            intakeSetpoint ?: intakePosition
        )
    }
}