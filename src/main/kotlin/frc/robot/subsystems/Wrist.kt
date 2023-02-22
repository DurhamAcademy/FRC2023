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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.PI

class Wrist : SubsystemBase() {
    private val motor = CANSparkMax(
        Constants.wrist.motor.id,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(Constants.wrist.motor.currentLimit)
        inverted = Constants.wrist.motor.inverted
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

    private val encoder = CANCoder(Constants.wrist.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
        configMagnetOffset(Constants.wrist.encoder.offset)
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
        else encoder.absolutePosition * ((2 * PI) / 360.0)
    val velocity: Double
        get() = encoder.velocity
    var setpoint: Double? = null
    var voltage: Double = 0.0
        set(value) {
            if (RobotBase.isSimulation()) simWristSystem.setInputVoltage(voltage)
            else motor.setVoltage(voltage)
            field = value
        }

    fun setPosition(position: Double) {
        setpoint = position
    }

    override fun periodic() {
        if (setpoint != null) {
            val output = pid.calculate(position, setpoint!!)
            setVoltage(output)
        }
    }

    override fun simulationPeriodic() {
        simWristSystem.update(0.02)
    }
}