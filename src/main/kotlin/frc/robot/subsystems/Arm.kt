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
import java.lang.Math.PI
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
        Constants.arm.armMass,
        true
    )

    val armEncoder = CANCoder(Constants.arm.encoder.id).apply {
        configFactoryDefault()
        configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
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
    val armFeedForward = ArmFeedforward(
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

    override fun periodic() {
        // SmartDashboard stuff
        SmartDashboard.putNumber("arm/Position", armPosition)
        SmartDashboard.putNumber("arm/Velocity", armVelocity)
        SmartDashboard.putNumber("arm/Setpoint", armSetpoint ?: -1000.0)
        if(armWristCheck(wrist.position)){
            armMotor.setVoltage(0.0)
        }
    }

    override fun simulationPeriodic() {
        simArmSystem.update(0.02)
    }
}