package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.Constants
import frc.robot.constants.Constants.DRIVE_GEAR_RATIO
import frc.robot.constants.Constants.WHEEL_CIRCUMFRENCE
import kotlin.math.PI

class SwerveModule(
    driveMotorId: Int,
    turnMotorId: Int,
    encoderId: Int,
    val mname: String,
    val location: Translation2d,
    angleZero: Double = 0.0,
) : SubsystemBase() {
    val stateEntry = getTab("Swerve Diagnostics")
        .add("$mname State v", 0)
        .entry
    val driveMotor = WPI_TalonFX(driveMotorId).apply {
        configFactoryDefault()
//        configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40.0, 45.0, 0.0))
        //driveMotor.configClosedloopRamp(0.25);
        configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 45.0, 0.5))
        setNeutralMode(NeutralMode.Brake)
        // Configure the encoders for both motors
//        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
        // set status frames
        setStatusFramePeriod(StatusFrame.Status_1_General, 10)
        setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255)
        setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10)
        setStatusFramePeriod(StatusFrame.Status_6_Misc, 10)
        setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 10)
        setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255)
        setStatusFramePeriod(StatusFrame.Status_10_Targets, 255)
        setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255)
        setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255)
        setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255)
        setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255)
    }
    private val turnMotor = WPI_TalonFX(turnMotorId).apply {
        // TODO:  ctrl click on configFactory default and check if the timeout is too long
        configFactoryDefault()
        // set status frames
        setStatusFramePeriod(StatusFrame.Status_1_General, 10)
        setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255)
        setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10)
        setStatusFramePeriod(StatusFrame.Status_6_Misc, 10)
        setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 10)
        setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255)
        setStatusFramePeriod(StatusFrame.Status_10_Targets, 255)
        setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255)
        setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255)
        setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255)
        setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255)

        configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 30.0, 35.0, 0.5))
        setNeutralMode(NeutralMode.Brake)
    }
    private val turnEncoder = CANCoder(encoderId).apply {
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-1 * angleZero)
        setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100)
        setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100, 100)
        configSensorDirection(false)
    }

    // smartdashboard
    // motors
    val tab = getTab("$mname Swerve Diagnostics")
    val driveMotorEntry = tab.add("$mname Drive Motor", 0.0).entry
    val turnMotorEntry = tab.add("$mname Turn Motor", 0.0).entry
    val turnEncoderEntry = tab.add("$mname Turn Encoder", 0.0).entry

    // motor sets
    val driveMotorVoltageEntry = tab.add("$mname Drive Motor Voltage", 0.0).entry
    val turnMotorVoltageEntry = tab.add("$mname Turn Motor Voltage", 0.0).entry

    // location/angle
    val positionEntry = tab.add("$mname Position", 0.0).entry
    val angleEntry = tab.add("$mname Angle", 0.0).entry


    val drivePid = ProfiledPIDController(
        Constants.DRIVE_P,
        Constants.DRIVE_I,
        Constants.DRIVE_D,
        TrapezoidProfile.Constraints(25.0, 1000.0)// TODO: Fix these
    )
    private val driveFF = SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA)


    val anglePid = ProfiledPIDController(
        Constants.ANGLE_P,
        Constants.ANGLE_I,
        Constants.ANGLE_D,
        TrapezoidProfile.Constraints(
            PI * 4,
            PI * 8
        )// TODO: Fix these
    ).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }
    private val angleFF = SimpleMotorFeedforward(Constants.angleKS, Constants.angleKV, Constants.angleKA)


//    class SwerveModuleSetpoint(
//        var driveSetpoint: Double?,
//        var angleSetpoint: Rotation2d?,
//    ) : SwerveModuleState(
//        driveSetpoint ?: 0.0,
//        angleSetpoint ?: Rotation2d()
//    ) {
//        constructor(state: SwerveModuleState) : this(state.speedMetersPerSecond, state.angle)
//        constructor() : this(null, null)
//    }
    val swerveModulePosition: SwerveModulePosition
        get() = SwerveModulePosition(
            driveMotor.selectedSensorPosition / 2048.0 * WHEEL_CIRCUMFRENCE / DRIVE_GEAR_RATIO,
            Rotation2d(MathUtil.angleModulus(degreesToRadians(turnEncoder.absolutePosition)))
        )

    @Suppress("RedundantSetter")
    val currentPosition: SwerveModuleState
        get() = SwerveModuleState(
            (driveMotor.selectedSensorVelocity * 10.0 / 2048.0 * WHEEL_CIRCUMFRENCE / DRIVE_GEAR_RATIO),
            Rotation2d(MathUtil.angleModulus(degreesToRadians(turnEncoder.absolutePosition)))
        )
    var setpoint = SwerveModuleState()
        set(value) {
            field = SwerveModuleState.optimize(value, currentPosition.angle)
        }
    var powerSaveMode = false
        set(value) {
            return
            field = value
            if (value) {
                driveMotor.setNeutralMode(NeutralMode.Coast)
                turnMotor.setNeutralMode(NeutralMode.Coast)
                driveMotor.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(
                    true,
                    10.0,
                    20.0,
                    0.0
                ))
            } else {
                driveMotor.setNeutralMode(NeutralMode.Brake)
                turnMotor.setNeutralMode(NeutralMode.Brake)
                driveMotor.configSupplyCurrentLimit(
                    SupplyCurrentLimitConfiguration(
                        true,
                        40.0,
                        45.0,
                        0.0
                    )
                )
            }
        }

    val previousDriveVoltage = 0.0
    val previousTurnVoltage = 0.0
    private fun setMotorVoltage(drive: Double, angle: Double) {
        if (drive != previousDriveVoltage) {
            driveMotor.setVoltage(drive)
        }
        if (angle != previousTurnVoltage) {
            turnMotor.setVoltage(angle)
        }

        // Shuffleboard
        driveMotorVoltageEntry.setDouble(drive)
        turnMotorVoltageEntry.setDouble(angle)
    }

    var lastTime = Timer.getFPGATimestamp()
    fun move() {
        // if encoder is not updating, stop the motor
        val drivePower =
            drivePid.calculate(
                currentPosition.speedMetersPerSecond,
                setpoint.speedMetersPerSecond
            ) + driveFF.calculate(
                drivePid.setpoint.position,
                drivePid.setpoint.velocity
            )
        val lastVel = anglePid.setpoint.velocity
        val t = Timer.getFPGATimestamp()
        val dt = t - lastTime
        val anglePower = -(anglePid.calculate(
            degreesToRadians(this.turnEncoder.absolutePosition),
            setpoint.angle.radians
        ) + angleFF.calculate(
            lastVel,
            anglePid.setpoint.velocity,
            0.02//minOf(dt,0.001) // stop possible divide by zero?
        ))
        setMotorVoltage(drivePower, anglePower)
        lastTime = t
    }

    var lastMotorTimestamp = 0.0
    override fun periodic() {
        move()
        SmartDashboard.putNumber("$mname Encoder", turnEncoder.absolutePosition)

        // simulate motor velocity based on motor percent output
//        val dt = Timer.getFPGATimestamp() - lastPeriodicTime
//        val vel = (driveMotor.selectedSensorVelocity + (driveMotor.motorOutputVoltage * 2048.0 / 12.0) * 0.02).toInt()
//        driveMotor.simCollection.setIntegratedSensorVelocity(vel)
//        driveMotor.simCollection.setIntegratedSensorRawPosition((vel * 2048.0 * dt).toInt())
//        move()
//        // SmartDashboard
        driveMotorEntry.setDouble(driveMotor.selectedSensorVelocity)
        turnMotorEntry.setDouble(turnMotor.selectedSensorVelocity)
        turnEncoderEntry.setDouble(turnEncoder.absolutePosition)
        positionEntry.setDouble(currentPosition.speedMetersPerSecond)
        angleEntry.setDouble(currentPosition.angle.degrees)
//
        stateEntry.setDouble(currentPosition.speedMetersPerSecond)
    }

}
