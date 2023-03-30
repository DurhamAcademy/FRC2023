package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode.kBrake
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.util.Units.rotationsToRadians
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.intake
import kotlin.math.PI

class Intake(
    var arm: Arm,
) : SubsystemBase() {

    private val intakeMotor = CANSparkMax(
        intake.driveMotorId,
        kBrushless
    ).apply {
        setSmartCurrentLimit(intake.driveMotorLimit) // add current limit to limit the torque
        idleMode = kBrake
    }

    private val modeMotor = CANSparkMax(
        intake.modeMotorId,
        kBrushless
    ).apply {
        setSmartCurrentLimit(intake.modeMotorLimit) // add current limit to limit the torque
        idleMode = kBrake
    }

    private val modeEncoder = modeMotor.encoder.apply {
        positionConversionFactor = 1/20.0
    }

    var modeZeroed = false

    fun zeroModeMotor() {
        modeEncoder.position = 0.0
        modeZeroed = true
    }


    private val deployMotor = CANSparkMax(
        intake.deployMotor.id,
        kBrushless
    ).apply {
        setSmartCurrentLimit(intake.systemMotorLimit) // add current limit to limit the torque
        idleMode = kBrake
    }

    private val deployEncoder = deployMotor.getAbsoluteEncoder(
        kDutyCycle
    ).apply {
        this.inverted = true
        this.setZeroOffset(0.1)
    }
    var intakePercentage: Double
        get() = intakeMotor.get()
        set(value) {
            deployMotor.set(value)
        }
    val limitSwitch = DigitalInput(
        intake.limitSwitch.intakeLimitSwitch
    )

    val deployPID = ProfiledPIDController(
        intake.deployMotor.kP,
        intake.deployMotor.kI,
        intake.deployMotor.kD,
        TrapezoidProfile.Constraints(
            intake.deployMotor.maxVelocity,
            intake.deployMotor.maxAcceleration
        )
    ).apply {
        setTolerance(
            intake.deployMotor.positionTolerance,
            intake.deployMotor.velocityTolerance
        )
        enableContinuousInput(0.0, PI * 2)
    }

    val modePID = ProfiledPIDController(
        intake.modeMotor.kP,
        intake.modeMotor.kI,
        intake.modeMotor.kD,
        TrapezoidProfile.Constraints(
            intake.deployMotor.maxVelocity,
            intake.deployMotor.maxAcceleration
        )
    ).apply {
        setTolerance(
            intake.deployMotor.positionTolerance,
            intake.deployMotor.velocityTolerance
        )
    }

    val limitSwitchPressed: Boolean
        get() = !limitSwitch.get()

    //@TODO Setter

    val tab = Shuffleboard.getTab("Intake")
    val driveMotorCurrent = tab.add("Motor Current jerguewf", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry
    val modeMotorCurrent = tab.add("Motor Current lajshg", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry
    val systemMotorCurrent = tab.add("Motor Current ais;hljk", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .entry

    val voltage: Double
        get() = modeMotor.busVoltage

    private var modePositionSetpoint: Double = 0.0

    private var deployPositionSetpoint: Double = 0.0
    val deployPosition: Double
        get() = rotationsToRadians(deployEncoder.position)

    val modePosition: Double
        get() = rotationsToRadians(modeEncoder.position)

    var modeVoltage = 0.0 // voltage while zeroing

    val modeCurrent
        get() = modeMotor.outputCurrent

    override fun periodic() {
        driveMotorCurrent.setDouble(intakeMotor.outputCurrent)
        modeMotorCurrent.setDouble(modeMotor.outputCurrent)
        systemMotorCurrent.setDouble(deployMotor.outputCurrent)
        SmartDashboard.putNumber("intake_pos", deployEncoder.position)
        SmartDashboard.putNumber("mode_pos", modePosition)

        val deployVoltage = deployPID.calculate(
            deployPosition,
            deployPositionSetpoint
        )
        deployMotor.setVoltage(deployVoltage.coerceIn(-6.0, 6.0))

        val modeV = if(modeZeroed) modePID.calculate(
            modePosition,
            modePositionSetpoint
        ) else modeVoltage
        modeMotor.setVoltage(modeV.coerceIn(-3.0, 3.0))
    }

    fun setModeAngle(angle: Double) {
        modePositionSetpoint = angle
    }

    fun setDeployAngle(angle: Double) {
        modePositionSetpoint = angle
    }
}