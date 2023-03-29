package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode.kBrake
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units.radiansToRotations
import edu.wpi.first.math.util.Units.rotationsToRadians
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
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

    private val cubeArmMotor = CANSparkMax(
        intake.modeMotorId,
        kBrushless
    ).apply {
        setSmartCurrentLimit(intake.modeMotorLimit) // add current limit to limit the torque
        idleMode = kBrake
        pidController.p = intake.modeMotor.kP
        pidController.i = intake.modeMotor.kI
        pidController.d = intake.modeMotor.kD
        pidController.setSmartMotionAccelStrategy(
            SparkMaxPIDController.AccelStrategy.kTrapezoidal,
            0
        )
        pidController.setSmartMotionMaxAccel(intake.modeMotor.maxAcceleration, 0)
        pidController.setSmartMotionMaxVelocity(intake.modeMotor.maxVelocity, 0)
        pidController.setSmartMotionMinOutputVelocity(0.0, 0)
        pidController.setSmartMotionAllowedClosedLoopError(0.0, 0)
        this.encoder.setPositionConversionFactor(1 / 30.0)
    }
    private val deployMotor = CANSparkMax(
        intake.deployMotor.id,
        kBrushless
    ).apply {
        setSmartCurrentLimit(intake.systemMotorLimit) // add current limit to limit the torque
        idleMode = kBrake
        pidController.p = intake.deployMotor.kP
        pidController.i = intake.deployMotor.kI
        pidController.d = intake.deployMotor.kD
        pidController.setSmartMotionAccelStrategy(
            SparkMaxPIDController.AccelStrategy.kTrapezoidal,
            0
        )
        pidController.setSmartMotionMaxAccel(intake.deployMotor.maxAcceleration, 0)
        pidController.setSmartMotionMaxVelocity(intake.deployMotor.maxVelocity, 0)
        pidController.setSmartMotionMinOutputVelocity(0.0, 0)
        pidController.setSmartMotionAllowedClosedLoopError(0.0, 0)
    }

    private val objectEngagementAlternateEncoder = deployMotor.getAbsoluteEncoder(
        kDutyCycle
    ).apply {
        this.inverted = true
        this.setZeroOffset(radiansToRotations(deployOffset))
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

    val cubeArmPID = ProfiledPIDController(
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
    }

    val limitSwitchPressed: Boolean
        get() = !limitSwitch.get()

    var modeMotorPosition: Double
        get() = Math.toRadians(objectEngagementAlternateEncoder.position)
        set(value) {
            objectEngagementAlternateEncoder.setZeroOffset(Math.toDegrees(value) - objectEngagementAlternateEncoder.position)
        }

    private var systemMotorOffset = 0.0
    var systemMotorPosition: Double
        get() = Math.toRadians(deployMotor.encoder.position) + systemMotorOffset
        set(value) {
            systemMotorOffset = value - Math.toRadians(deployMotor.encoder.position)
        }
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
        get() = cubeArmMotor.busVoltage

    private var cubeArmPositionSetpoint: Double = PI / 2

    private var deployPositionSetpoint: Double = 0.0

    private var deployOffset = -.7
    var deployPosition: Double
        get() = rotationsToRadians(objectEngagementAlternateEncoder.position)
        set(value) {
            intakeOffset = rotationsToRadians(objectEngagementAlternateEncoder.position)
        }

    private var intakeOffset = 0.0
    var cubeArmPosition: Double
        get() = rotationsToRadians(cubeArmMotor.encoder.position)
        set(value) {
            cubeArmMotor.encoder.setPosition(radiansToRotations(value))
        }

    override fun periodic() {
        driveMotorCurrent.setDouble(intakeMotor.outputCurrent)
        modeMotorCurrent.setDouble(cubeArmMotor.outputCurrent)
        systemMotorCurrent.setDouble(deployMotor.outputCurrent)

        val deployVoltage = deployPID.calculate(
            deployPosition,
            deployPositionSetpoint
        )
        deployMotor.setVoltage(deployVoltage.coerceIn(-0.5, 0.5))

        val intakeVoltage = cubeArmPID.calculate(
            cubeArmPosition,
            cubeArmPositionSetpoint
        )
        cubeArmMotor.setVoltage(intakeVoltage.coerceIn(-0.5, 0.5))
    }

    fun setCubeArmAngle(angle: Double) {
        cubeArmPositionSetpoint = angle
    }

    fun setDeployAngle(angle: Double) {
        cubeArmPositionSetpoint = angle
    }
}