package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode.kBrake
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import com.revrobotics.SparkMaxAlternateEncoder.Type.kQuadrature
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.math.util.Units.rotationsToRadians
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.intake

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
    private val objectEngagementAlternateEncoder = intakeMotor.getAlternateEncoder(
        kQuadrature,
        intake.driveMotorEncoderCPR
    )
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

    var intakePercentage: Double
        get() = intakeMotor.get()
        set(value) {
            intakeMotor.set(value)
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
            objectEngagementAlternateEncoder.position = Math.toDegrees(value)
        }

    private var systemMotorOffset = 0.0
    var systemMotorPosition: Double
        get() = Math.toRadians(deployMotor.encoder.position) + systemMotorOffset
        set(value) {
            systemMotorOffset = value - Math.toRadians(deployMotor.encoder.position)
        }
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

    val voltage: Double
        get() = cubeArmMotor.busVoltage

    private var intakeSetpoint: Double? = null

    private var intakePositionSetpoint: Double
        get() = intakeSetpoint ?: intakePosition
        set(value) {
            intakeSetpoint = value.coerceIn(
                intake.minAngle,
                intake.maxAngle
            )
        }

    private var deployOffset = 0.0
    var deployPosition: Double
        get() = rotationsToRadians(deployMotor.encoder.position) + deployOffset
        set(value) {
            intakeOffset = value - rotationsToRadians(deployMotor.encoder.position)
        }

    private var intakeOffset = 0.0
    var intakePosition: Double
        get() = rotationsToRadians(objectEngagementAlternateEncoder.position) + intakeOffset
        set(value) {
            intakeOffset = value - rotationsToRadians(objectEngagementAlternateEncoder.position)
        }

    override fun periodic() {
        driveMotorCurrent.setDouble(intakeMotor.outputCurrent)
        modeMotorCurrent.setDouble(cubeArmMotor.outputCurrent)
        systemMotorCurrent.setDouble(deployMotor.outputCurrent)

        if (intake.deployMotor.pidOnRio) {
            deployMotor.setVoltage(deployPID.calculate(deployPosition, intakePositionSetpoint))
        } else {
            // todo: set deploy motor to position
        }

        if (intake.modeMotor.pidOnRio) {
            cubeArmMotor.setVoltage(cubeArmPID.calculate(intakePosition, intakePositionSetpoint))
        } else {
            // todo: set cube arm motor to position
        }
    }
}