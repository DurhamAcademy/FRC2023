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
        arm.motor.kP,
        arm.motor.kI,
        arm.motor.kD,
        TrapezoidProfile.Constraints(
            arm.motor.maxVelocity,
            arm.motor.maxAcceleration
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
    val ArmTab = Shuffleboard.getTab("Arm")
    val ArmMotorVoltage = ArmTab.add("Arm Motor Voltage", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -12.0, "max" to 12.0)
        )
        .entry
    val ArmMotorPosition = ArmTab.add("Arm Motor Setpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -1.0, "max" to 1.0)
        )
        .entry
    val ArmFFMotorPosition = ArmTab.add("ArmFFMotorPosition", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -12.0, "max" to 12.0)
        )
        .entry
    val ArmPIDMotorPosition = ArmTab.add("ArmPIDMotorPosition", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to -12.0, "max" to 12.0)
        )
        .entry
    val dashSetpoint = ArmTab.add("dashSetpoint", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf("min" to arm.minAngle, "max" to arm.maxAngle)
        )
        .entry

    override fun periodic() {

        SmartDashboard.putNumber("arm/POS", armPosition)
        SmartDashboard.putNumber("arm/SP", armSetpoint ?: -99999.0)

        val calculate = armPID.calculate(
            armPosition,
            armSetpoint ?: armPosition
        )
        val FF = armFeedForward.calculate(
            // convert from 0 being horizontal arm to 0 being upright
            armPosition + (PI / 2),
            armPID.setpoint.velocity
        )

        val voltage = if (armSetpoint != null) {
            FF + calculate
        } else 0.0
        setArmVoltage(voltage)

        // Shuffleboard stuff
        ArmMotorVoltage.setDouble(voltage)

        SmartDashboard.putNumber("arm/SPVel", armPID.setpoint.velocity)
        SmartDashboard.putNumber("arm/SPErr", armPID.setpoint.position)

        ArmFFMotorPosition.setDouble(FF)
        ArmPIDMotorPosition.setDouble(calculate)


        // SmartDashboard stuff
        SmartDashboard.putNumber("arm/Position", armPosition)
        SmartDashboard.putNumber("arm/Velocity", armVelocity)
        SmartDashboard.putNumber("arm/Setpoint", armSetpoint ?: -1000.0)
    }

    override fun simulationPeriodic() {
        simArmSystem.update(0.02)
    }
}