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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.PI

class Wrist : SubsystemBase() {
    private val wristMotor = CANSparkMax(
        Constants.wrist.motor.id,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(Constants.wrist.motor.currentLimit)
        inverted = Constants.wrist.motor.inverted
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    private val simWristSystem = SingleJointedArmSim(
        DCMotor.getNEO(1),
        Constants.wrist.motor.gearRatio,
        Constants.wrist.momentOfInertia,
        Constants.wrist.simArmLength,
        Constants.wrist.minAngle,
        Constants.wrist.maxAngle,
        Constants.wrist.armMass,
        true
    )

    private val wristEncoder = CANCoder(Constants.wrist.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-Constants.wrist.encoder.offset)
        configSensorDirection(Constants.wrist.encoder.inverted)
    }
    private val pid = ProfiledPIDController(
        Constants.wrist.motor.kP,
        Constants.wrist.motor.kI,
        Constants.wrist.motor.kD,
        TrapezoidProfile.Constraints(
            Constants.wrist.motor.maxVelocity,
            Constants.wrist.motor.maxAcceleration
        )
    ).apply {
        setTolerance(
            Constants.wrist.motor.positionTolerance,
            Constants.wrist.motor.velocityTolerance
        )
    }

    // probably wont need a feedforward for the wrist because its not carrying
    // much weight and can hold it's position with brake mode alone.
    val feedForward = ArmFeedforward(
        Constants.wrist.motor.kS,
        Constants.wrist.motor.kG,
        Constants.wrist.motor.kV,
        Constants.wrist.motor.kA
    )
    val position: Double
        get() = if (RobotBase.isSimulation()) simWristSystem.angleRads
        else wristEncoder.absolutePosition * ((2 * PI) / 360.0)
    val velocity: Double
        get() = wristEncoder.velocity
    var setpoint: Double? = null
    var voltage: Double = 0.0
        set(value) {
            if (RobotBase.isSimulation()) simWristSystem.setInputVoltage(voltage)
            else wristMotor.setVoltage(voltage)
            field = value
        }

    fun setPosition(position: Double) {
        setpoint = position
    }
    val wristVelocity: Double
        get() = wristEncoder.velocity
    var wristSetpoint: Double? = null
    fun setWristVoltage(voltage: Double) {
        if (RobotBase.isSimulation()) simWristSystem.setInputVoltage(voltage)
        else wristMotor.setVoltage(voltage)
    }

    override fun periodic() {
        val output = pid.calculate(position, setpoint ?: position)
        SmartDashboard.putNumber("wrist/position", position)
        SmartDashboard.putNumber("wrist/velocity", velocity)
        SmartDashboard.putNumber("wrist/setpoint", setpoint ?: position)
        SmartDashboard.putNumber("wrist/output", output)
        if (setpoint != null) {
            voltage = output
        } else voltage = 0.0
    }

    override fun simulationPeriodic() {
        simWristSystem.update(0.02)
    }
}