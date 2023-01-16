package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab
import edu.wpi.first.wpilibj2.command.SubsystemBase

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
    private val driveMotor = WPI_TalonFX(driveMotorId).apply {
        configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40.0, 45.0, 0.5)) //40, 45, 0.5
        //driveMotor.configClosedloopRamp(0.25);
        configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 45.0, 0.5))
        setNeutralMode(NeutralMode.Brake)

        // Configure the encoders for both motors
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
    }
    private val turnMotor = WPI_TalonFX(turnMotorId).apply {
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255)
        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255)
        setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255)
        setNeutralMode(NeutralMode.Brake)
    }
    private val turnEncoder = CANCoder(encoderId).apply {
        configMagnetOffset(-1 * angleZero)
        configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100)
    }

    //should adjust these gains or characterize since they are a little slow
    val DRIVE_P = 0.1
    val DRIVE_I = 0.0
    val DRIVE_D = 0.0
    val drivePid = ProfiledPIDController(
        DRIVE_P,
        DRIVE_I,
        DRIVE_D,
        TrapezoidProfile.Constraints(2.0, 2.0)// TODO: Fix these
    )
    //should adjust these gains or characterize since they are a little slow
    val ANGLE_P = 0.01
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
        driveMotor.selectedSensorVelocity,
        Rotation2d(turnEncoder.position)
    )
        private set(value) {
            field = value
        }

    private fun setMotorSpeed(drive: Double, angle: Double) {
        driveMotor.set(drive)
        turnMotor.set(angle)
    }

    private fun move() {
        anglePid.setGoal((setpoint.angleSetpoint ?: currentPosition.angle).radians)
        drivePid.setGoal(setpoint.driveSetpoint ?: currentPosition.speedMetersPerSecond)
        val anglePower = anglePid.calculate(currentPosition.angle.radians)
        val drivePower = drivePid.calculate(currentPosition.speedMetersPerSecond)
        setMotorSpeed(drivePower, anglePower)
    }


    override fun periodic() {
        move()
    }

    fun resetEncoders() {
        driveMotor.selectedSensorPosition = 0.0
        turnEncoder.position = 0.0
    }
}