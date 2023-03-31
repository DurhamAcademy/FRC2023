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
import frc.kyberlib.command.Game
import frc.robot.constants.ArmConstants
import java.lang.Math.PI
import java.lang.Math.toRadians

class Arm : SubsystemBase() {
    private val armMotor = CANSparkMax(
        ArmConstants.Motor.id,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(ArmConstants.Motor.currentLimit)
        inverted = ArmConstants.Motor.inverted
        this.serialNumber
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 255)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 255)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 255)
        setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 255)
    }
    private val simArmSystem = SingleJointedArmSim(
        DCMotor.getNEO(1),
        ArmConstants.Motor.gearRatio,
        ArmConstants.momentOfInertia * 0.001,
        ArmConstants.length * 0.5,
        ArmConstants.minAngle,
        ArmConstants.maxAngle,
        true
    )

    private val armEncoder = CANCoder(ArmConstants.Encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-ArmConstants.Encoder.offset)
        configSensorDirection(ArmConstants.Encoder.inverted)
    }
    private val armPID = ProfiledPIDController(
        ArmConstants.Motor.kP,
        ArmConstants.Motor.kI,
        ArmConstants.Motor.kD,
        TrapezoidProfile.Constraints(
            ArmConstants.Motor.maxVelocity,
            ArmConstants.Motor.maxAcceleration
        )
    ).apply {
        setTolerance(
            ArmConstants.Motor.positionTolerance,
            ArmConstants.Motor.velocityTolerance
        )
    }
    private var armFeedForward = ArmFeedforward(
        ArmConstants.Motor.kS,
        ArmConstants.Motor.kG,
        ArmConstants.Motor.kV,
        ArmConstants.Motor.kA
    )

    /**
     * @return angle in radians. 0 is upright, -pi/2 is horizontal where the arm
     * is over our intake. pi/2 is horizontal where the arm out of frame
     * perimeter.
     */
    val armPosition: Double
        get() = if (RobotBase.isSimulation()) armPID.setpoint.position//simArmSystem.angleRads
        else toRadians(armEncoder.absolutePosition)
    private var armSetpoint: Double? = null

    private var lastVoltage = 0.0
    private fun setArmVoltage(voltage: Double) {
        if (Game.sim) simArmSystem.setInputVoltage(voltage)
        else
            if (lastVoltage != voltage) {
                armMotor.setVoltage(voltage)
                lastVoltage = voltage
            }
    }

    /**
     * @param position angle in radians. 0 is upright, -pi/2 is horizontal over
     * our intake.
     */
    fun setArmPosition(position: Double) {
        armSetpoint = position.coerceIn(
            ArmConstants.minAngle,
            ArmConstants.maxAngle
        )
    }

    // shuffleboard
    private val armTab = Shuffleboard.getTab("ArmConstants")
    private val voltageEntry = armTab.add("ArmConstants Motor Voltage", 0.0)
        .withWidget(BuiltInWidgets.kVoltageView)
        .withProperties(
            mapOf("min" to -12.0, "max" to 12.0)
        )
        .withPosition(2, 0)
        .withSize(4, 2)
        .entry

    private val positionEntry = armTab.add("Position", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(
            mapOf("min" to ArmConstants.minAngle, "max" to ArmConstants.maxAngle)
        )
        .withPosition(2, 2)
        .withSize(4, 4)
        .entry

    private val currentSetpointEntry = armTab.add("Current Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(
            mapOf("min" to ArmConstants.minAngle, "max" to ArmConstants.maxAngle)
        )
        .withPosition(2, 6)
        .withSize(4, 4)
        .entry

    override fun periodic() {

        SmartDashboard.putNumber("arm/POS", armPosition)
        SmartDashboard.putNumber("arm/SP", armSetpoint ?: -99999.0)

        val calculate = armPID.calculate(
            armPosition,
            armSetpoint ?: armPosition
        )
        val feedForwardPower = armFeedForward.calculate(
            // convert from 0 being horizontal arm to 0 being upright
            armPosition + (PI / 2),
            armPID.setpoint.velocity
        )

        val voltage = if (armSetpoint != null) {
            feedForwardPower + calculate
        } else 0.0
        setArmVoltage(voltage)

        // Shuffleboard stuff
        voltageEntry.setDouble(voltage)
        positionEntry.setDouble(armPosition)
        currentSetpointEntry.setDouble(armPID.setpoint.position)
    }

    override fun simulationPeriodic() {
        simArmSystem.update(0.02)
    }

}