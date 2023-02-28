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
//import frc.robot.utils.armFeedforward
//import frc.robot.utils.createArmSystemPlant
import java.lang.Math.PI
import java.lang.Math.toRadians
import javax.swing.text.Position

class Arm(
    var wrist: Wrist

) : SubsystemBase() {
    fun armWristCheck(position: Double): Boolean {
        if(position == 0.0){
            return true
        }
        return false
    }

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
        true
    )

    val armEncoder = CANCoder(Constants.arm.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-Constants.arm.encoder.offset)
        configSensorDirection(Constants.arm.encoder.inverted)
    }
    val armPID = ProfiledPIDController(
        5.0,
        0.0,
        0.5,
        TrapezoidProfile.Constraints(
            1.0,
            1.0
        )
    )
    var armFeedForward = ArmFeedforward(
        Constants.arm.motor.kS,
        Constants.arm.motor.kG,
        Constants.arm.motor.kV,
        Constants.arm.motor.kA
    )

    val armPosition: Double
        get() = if (RobotBase.isSimulation()) simArmSystem.angleRads
        else toRadians(armEncoder.absolutePosition)
    val armVelocity: Double
        get() = toRadians(armEncoder.velocity)
    var armSetpoint: Double? = null

    fun setArmVoltage(voltage: Double) {
        if (RobotBase.isSimulation()) simArmSystem.setInputVoltage(voltage)
        else armMotor.setVoltage(voltage)
    }

    fun reset() {
        armPID.reset(armPosition)
        println("RESET")
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
            mapOf("min" to -PI, "max" to PI)
        )
        .entry

    fun armWristSafety(): Boolean {
        return wrist.position > 30 || wrist.position < -30
    }

    override fun periodic() {
        SmartDashboard.putNumber("arm/POS", armPosition)
        SmartDashboard.putNumber("arm/SP", armSetpoint ?: -99999.0)

        val calculate = armPID.calculate(
            armPosition,
            armSetpoint ?: 0.0
        )
        var FF = armFeedForward.calculate(
            // convert from 0 being horizontal arm to 0 being upright
            armPosition + (PI / 2),
            armPID.setpoint.velocity
        )

        val voltage = if (armSetpoint != null) {
            FF + calculate
        } else 0.0
        setArmVoltage(voltage.coerceIn(-4.0, 4.0))

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

        if(armWristSafety()){
            setArmVoltage(0.0)
        }

        if(armWristCheck(wrist.position)){
            armMotor.setVoltage(0.0)
        }
    }

    override fun simulationPeriodic() {
        simArmSystem.update(0.02)
    }

    /**
     * @param position angle in radians. 0 is upright, -pi/2 is horizontal over
     * our intake.
     */
    fun setArmPosition(position: Double) {
        armSetpoint = position

    }
}