package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DRIVE_GEAR_RATIO
import frc.robot.Constants.WHEEL_CIRCUMFRENCE

class SwerveModule(
    driveMotorId: Int,
    turnMotorId: Int,
    encoderId: Int,
    name: String,
    val position: Translation2d,
    angleZero: Double = 0.0,
) : SubsystemBase() {
    val stateEntry = getTab("Swerve Diagnostics")
        .add("$name State v", 0)
        .entry
    private val driveMotor = TalonFX(driveMotorId).apply {
        configFactoryDefault()
        configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40.0, 45.0, 0.0)) // why 0.5?
        //driveMotor.configClosedloopRamp(0.25);
        configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 45.0, 0.5))
        setNeutralMode(NeutralMode.Brake)

        // Configure the encoders for both motors
//        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
    }
    private val turnMotor = TalonFX(turnMotorId).apply {
        configFactoryDefault()
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255)
        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255)
        setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255)
        setNeutralMode(NeutralMode.Brake)
    }
    private val turnEncoder = CANCoder(encoderId).apply {
        configMagnetOffset(-1 * angleZero)
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100)

        configSensorDirection(false)
    }

    // smartdashboard
    // motors
    val tab = getTab("$name Swerve Diagnostics")
    val driveMotorEntry = tab.add("$name Drive Motor", 0.0).entry
    val turnMotorEntry = tab.add("$name Turn Motor", 0.0).entry
    val turnEncoderEntry = tab.add("$name Turn Encoder", 0.0).entry

    // motor sets
    val driveMotorSetEntry = tab.add("$name Drive Motor Set", 0.0).entry
    val turnMotorSetEntry = tab.add("$name Turn Motor Set", 0.0).entry

    // position/angle
    val positionEntry = tab.add("$name Position", 0.0).entry
    val angleEntry = tab.add("$name Angle", 0.0).entry

    val DRIVE_P = 0.1
    val DRIVE_I = 0.0
    val DRIVE_D = 0.0
    val drivePid = ProfiledPIDController(
        DRIVE_P,
        DRIVE_I,
        DRIVE_D,
        TrapezoidProfile.Constraints(2.0, 2.0)// TODO: Fix these
    )
    val ANGLE_P = 0.1
    val ANGLE_I = 0.0
    val ANGLE_D = 0.0
    val anglePid = ProfiledPIDController(
        ANGLE_P,
        ANGLE_I,
        ANGLE_D,
        TrapezoidProfile.Constraints(
            3.0,
            3.0
        )// TODO: Fix these
    ).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    class SwerveModuleSetpoint(
        var driveSetpoint: Double?,
        var angleSetpoint: Rotation2d?,
    ) : SwerveModuleState(
        driveSetpoint ?: 0.0,
        angleSetpoint ?: Rotation2d()
    ) {
        constructor(state: SwerveModuleState) : this(state.speedMetersPerSecond, state.angle)
        constructor() : this(null, null)
    }

    var setpoint = SwerveModuleSetpoint()

    @Suppress("RedundantSetter")
    var currentPosition = SwerveModuleState(
        (driveMotor.selectedSensorVelocity / 2048.0) * WHEEL_CIRCUMFRENCE * 10 / DRIVE_GEAR_RATIO,
        Rotation2d(MathUtil.angleModulus(Units.degreesToRadians(turnEncoder.position)))
    )
        private set(value) {
            field = value
        }

    private fun setMotorSpeed(drive: Double, angle: Double) {
        driveMotor.set(ControlMode.PercentOutput, drive)
        turnMotor.set(ControlMode.PercentOutput, angle)
        // SmartDashboard
        driveMotorSetEntry.setDouble(drive)
        turnMotorSetEntry.setDouble(angle)
    }

    private fun move() {
        anglePid.setGoal((setpoint.angleSetpoint ?: currentPosition.angle).radians)
        drivePid.setGoal(setpoint.driveSetpoint ?: currentPosition.speedMetersPerSecond)
        val anglePower = anglePid.calculate(currentPosition.angle.radians)
        val drivePower = drivePid.calculate(currentPosition.speedMetersPerSecond)
        setMotorSpeed(drivePower, anglePower)
    }
    val lastPeriodicTime = Timer.getFPGATimestamp()

    override fun periodic() {
        // simulate motor velocity based on motor percent output
        val dt = Timer.getFPGATimestamp() - lastPeriodicTime
        val vel = (driveMotor.selectedSensorVelocity + (driveMotor.motorOutputVoltage * 2048.0 / 12.0) * 0.02).toInt()
        driveMotor.simCollection.setIntegratedSensorVelocity(vel)
        driveMotor.simCollection.setIntegratedSensorRawPosition((vel * 2048.0 * dt).toInt())
        move()
        currentPosition = SwerveModuleState(
            (driveMotor.selectedSensorVelocity / 2048.0) * WHEEL_CIRCUMFRENCE * 10 / DRIVE_GEAR_RATIO,
            Rotation2d(MathUtil.angleModulus(Units.degreesToRadians(turnEncoder.position)))
        )
        // SmartDashboard
        driveMotorEntry.setDouble(driveMotor.selectedSensorVelocity)
        turnMotorEntry.setDouble(turnMotor.selectedSensorVelocity)
        turnEncoderEntry.setDouble(turnEncoder.position)
        positionEntry.setDouble(currentPosition.speedMetersPerSecond)
        angleEntry.setDouble(currentPosition.angle.radians)

        stateEntry.setDouble(currentPosition.speedMetersPerSecond)
    }

    fun resetEncoders() {
        driveMotor.selectedSensorPosition = 0.0
        turnEncoder.position = 0.0
    }
}