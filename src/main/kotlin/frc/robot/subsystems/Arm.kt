package frc.robot.subsystems

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import java.lang.Math.PI

class Arm : SubsystemBase() {
    val armMotor = CANSparkMax(
        Constants.arm.motor.id,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(Constants.arm.motor.currentLimit)
        inverted = Constants.arm.motor.inverted
    }
    val simArmSystem = SingleJointedArmSim(
        DCMotor.getNEO(1),
        Constants.arm.motor.gearRatio,
        Constants.arm.momentOfInertia,
        Constants.arm.armLength,
        Constants.arm.minAngle,
        Constants.arm.maxAngle,
        Constants.arm.armMass,
        true
    )

    val armEncoder = CANCoder(Constants.arm.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(Constants.arm.encoder.offset)
        configSensorDirection(Constants.arm.encoder.inverted)
    }
    val armPID = ProfiledPIDController(
        Constants.arm.motor.kP,
        Constants.arm.motor.kI,
        Constants.arm.motor.kD,
        TrapezoidProfile.Constraints(
            Constants.arm.motor.maxVelocity,
            Constants.arm.motor.maxAcceleration
        )
    ).apply {
        setTolerance(
            Constants.arm.motor.positionTolerance,
            Constants.arm.motor.velocityTolerance
        )
    }
    var armFeedForward = ArmFeedforward(
        Constants.arm.motor.kS,
        Constants.arm.motor.kG,
        Constants.arm.motor.kV,
        Constants.arm.motor.kA
    )
    val armPosition: Double
        get() = if (RobotBase.isSimulation()) simArmSystem.angleRads
        else armEncoder.absolutePosition * ((2 * PI) / 360.0)
    val armVelocity: Double
        get() = armEncoder.velocity
    var armSetpoint: Double? = null
    fun setArmVoltage(voltage: Double) {
        if (RobotBase.isSimulation()) simArmSystem.setInputVoltage(voltage)
        else armMotor.setVoltage(voltage)
    }

    fun setArmPosition(position: Double) {
        armSetpoint = position
    }

    // shuffleboard
    val ArmTab = Shuffleboard.getTab("Arm")
    val ArmMotorVoltage = ArmTab.add("Arm Motor Voltage", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -1.0, "max" to 1.0)
        )
        .getEntry()
    val ArmMotorPosition = ArmTab.add("Arm Motor Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -1.0, "max" to 1.0)
        )
        .entry
    override fun periodic() {
        val voltage = armFeedForward.calculate(
            // convert from 0 being horizontal arm to 0 being upright
            armPosition - (PI / 2), // FIXME: Test this
            armVelocity,
            armSetpoint ?: armPosition
        ) + armPID.calculate(
            armPosition,
            armSetpoint ?: armPosition
        )
        voltage.coerceIn(-1.0,1.0) // Fixme: remove this after testing
        setArmVoltage(voltage)

        // Shuffleboard stuff
        ArmMotorVoltage.setDouble(voltage)


        // SmartDashboard stuff
        SmartDashboard.putNumber("arm/Position", armPosition)
        SmartDashboard.putNumber("arm/Velocity", armVelocity)
        SmartDashboard.putNumber("arm/Setpoint", armSetpoint ?: -1000.0)
    }

    override fun simulationPeriodic() {
        simArmSystem.update(0.02)
    }
}