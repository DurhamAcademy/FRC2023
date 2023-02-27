package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DRIVE_GEAR_RATIO
import frc.robot.Constants.WHEEL_CIRCUMFRENCE

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
        configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40.0, 45.0, 0.0)) // why 0.5?
        //driveMotor.configClosedloopRamp(0.25);
        configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 45.0, 0.5))
        setNeutralMode(NeutralMode.Brake)
        // Configure the encoders for both motors
//        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
    }
    private val turnMotor = WPI_TalonFX(turnMotorId).apply {
        configFactoryDefault()
//        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255)
//        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255)
//        setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255)
        setNeutralMode(NeutralMode.Brake)
    }
    private val turnEncoder = CANCoder(encoderId).apply {
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        configMagnetOffset(-1 * angleZero)
//        setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100)

        configSensorDirection(false)

    }

    // smartdashboard
    // motors
    val tab = getTab("$mname Swerve Diagnostics")
    val driveMotorEntry = tab.add("$mname Drive Motor", 0.0).entry
    val turnMotorEntry = tab.add("$mname Turn Motor", 0.0).entry
    val turnEncoderEntry = tab.add("$mname Turn Encoder", 0.0).entry

    // motor sets
    val driveMotorSetEntry = tab.add("$mname Drive Motor Set", 0.0).entry
    val turnMotorSetEntry = tab.add("$mname Turn Motor Set", 0.0).entry

    // location/angle
    val positionEntry = tab.add("$mname Position", 0.0).entry
    val angleEntry = tab.add("$mname Angle", 0.0).entry


    val drivePid = ProfiledPIDController(
        Constants.DRIVE_P,
        Constants.DRIVE_I,
        Constants.DRIVE_D,
        TrapezoidProfile.Constraints(3.0, .0)// TODO: Fix these
    )
    private val driveFF = SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA)


    val anglePid = ProfiledPIDController(
        Constants.ANGLE_P,
        Constants.ANGLE_I,
        Constants.ANGLE_D,
        TrapezoidProfile.Constraints(
            32.0,
            64.0
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
            driveMotor.selectedSensorPosition / 2048.0 * WHEEL_CIRCUMFRENCE * 10 / DRIVE_GEAR_RATIO,
            Rotation2d(MathUtil.angleModulus(degreesToRadians(turnEncoder.absolutePosition)))
        )

    @Suppress("RedundantSetter")
    val currentPosition: SwerveModuleState
        get() = SwerveModuleState(
            (driveMotor.selectedSensorVelocity / 2048.0 * WHEEL_CIRCUMFRENCE / DRIVE_GEAR_RATIO),
            Rotation2d(MathUtil.angleModulus(degreesToRadians(turnEncoder.absolutePosition)))
        )
    var setpoint = SwerveModuleState()
        set(value) {
            field = SwerveModuleState.optimize(value, currentPosition.angle)
        }

    private fun setMotorSpeed(drive: Double, angle: Double) {
        driveMotor.setVoltage(drive)
        turnMotor.setVoltage(angle)
        // SmartDashboard
        driveMotorSetEntry.setDouble(drive)
        turnMotorSetEntry.setDouble(angle)
    }

    fun move() {
        val drivePower =
            drivePid.calculate(currentPosition.speedMetersPerSecond, setpoint.speedMetersPerSecond) + driveFF.calculate(
                drivePid.setpoint.position,
                drivePid.setpoint.velocity
            )
        SmartDashboard.putNumber("drivetrain/posset", drivePid.setpoint.position)
        SmartDashboard.putNumber("drivetrain/posvel", drivePid.setpoint.velocity)
        val anglePower = -(anglePid.calculate(
            degreesToRadians(this.turnEncoder.absolutePosition),
            setpoint.angle.radians
        ) + angleFF.calculate(anglePid.setpoint.velocity))
        setMotorSpeed(drivePower, anglePower)
    }

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
