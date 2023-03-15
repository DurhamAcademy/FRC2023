package frc.robot.subsystems

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.kyberlib.command.Game
import frc.robot.constants.arm
//import frc.robot.utils.armFeedforward
//import frc.robot.utils.createArmSystemPlant
import java.lang.Math.PI
import java.lang.Math.toRadians

class Arm : SubsystemBase() {
    val armMotor = CANSparkMax(
        arm.motor.id,
        CANSparkMaxLowLevel.MotorType.kBrushless
    ).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(arm.motor.currentLimit)
        inverted = arm.motor.inverted
        this.serialNumber
    }
    val simArmSystem = SingleJointedArmSim(
        DCMotor.getNEO(1),
        arm.motor.gearRatio,
        arm.momentOfInertia * 0.001,
        arm.length * 0.5,
        arm.minAngle,
        arm.maxAngle,
        true
    )

    val armEncoder = CANCoder(arm.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-arm.encoder.offset)
        configSensorDirection(arm.encoder.inverted)
    }
    val armPID = ProfiledPIDController(
        5.0,
        0.0,
        0.5,
        TrapezoidProfile.Constraints(
            3.0,
            2.5
        )
    ).apply {
        setTolerance(
            arm.motor.positionTolerance,
            arm.motor.velocityTolerance
        )
    }
    var armFeedForward = ArmFeedforward(
        arm.motor.kS,
        arm.motor.kG,
        arm.motor.kV,
        arm.motor.kA
    )
    var armOffset = Preferences.getDouble("armOffset", 0.0).apply {
        this@Arm.armEncoder
            .configMagnetOffset(-this + arm.encoder.offset)

    }
        set(value) {
            field = value
            if (Preferences.containsKey("armOffset")) {
                Preferences.setDouble("armOffset", value)
            } else {
                Preferences.initDouble("armOffset", value)
            }
            this@Arm.armEncoder
                .configMagnetOffset(-value + arm.encoder.offset)
        }

    val armPosition: Double
        get() = if (RobotBase.isSimulation()) armPID.setpoint.position//simArmSystem.angleRads
        else toRadians(armEncoder.absolutePosition)
    val armVelocity: Double
        get() = toRadians(armEncoder.velocity)
    private var armSetpoint: Double? = null
    fun setArmVoltage(voltage: Double) {
        if (Game.sim) simArmSystem.setInputVoltage(voltage)
        else armMotor.setVoltage(voltage)
    }

    fun reset() {
        armPID.reset(armPosition)
        println("RESET")
    }

    /**
     * @param position angle in radians. 0 is upright, -pi/2 is horizontal over
     * our intake.
     */
    fun setArmPosition(position: Double) {
        armSetpoint = position.coerceIn(
            arm.minAngle,
            arm.maxAngle
        )
    }

    // shuffleboard
    val armTab = Shuffleboard.getTab("Arm")
    val voltageEntry = armTab.add("Arm Motor Voltage", 0.0)
        .withWidget(BuiltInWidgets.kVoltageView)
        .withProperties(
            mapOf("min" to -12.0, "max" to 12.0)
        )
        .withPosition(2, 0)
        .withSize(4, 2)
        .entry

    val positionEntry = armTab.add("Position", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(
            mapOf("min" to arm.minAngle, "max" to arm.maxAngle)
        )
        .withPosition(2, 2)
        .withSize(4, 4)
        .entry

    val currentSetpointEntry = armTab.add("Current Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(
            mapOf("min" to arm.minAngle, "max" to arm.maxAngle)
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