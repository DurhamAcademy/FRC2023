// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot2.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.SensorInitializationStrategy
import com.ctre.phoenix.sensors.WPI_CANCoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot2.Constants.chassisConstants

class SwerveModule(
    driveMotorID: Int,
    driveMotorReversed: Boolean,
    turnMotorID: Int,
    turnMotorReversed: Boolean,
    absoluteEncoderID: Int,
    offset: Double,
    kP: Double,
    kI: Double,
    kD: Double,
) : SubsystemBase() {
    private val driveMotor: WPI_TalonFX = WPI_TalonFX(driveMotorID).apply{
        configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        setNeutralMode(NeutralMode.Brake)
        inverted = driveMotorReversed
    }
    init {

    }
    private val turnMotor: WPI_TalonFX = WPI_TalonFX(turnMotorID).apply {
        configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20)
        configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        inverted = turnMotorReversed
        setNeutralMode(NeutralMode.Brake)
        config_kP(0, kP)
        config_kI(0, kI)
        config_kD(0, kD)
    }

    // private final TalonFXSensorCollection driveMotorEnc;
    // private final TalonFXSensorCollection turnMotorEnc
    //private DigitalInput absoluteEnc;
    private val absoluteEncoderOffset: Double = offset
    private val absoluteEncoder: WPI_CANCoder = WPI_CANCoder(absoluteEncoderID).apply {
        configSensorDirection(false)
        configSensorDirection(false)
        configMagnetOffset(absoluteEncoderOffset)
    }
    var feedforward =
        SimpleMotorFeedforward(chassisConstants.kS, chassisConstants.kV, chassisConstants.kA) //Revise and

    val turnAngle: Rotation2d
        // public void resetEncoder(){
        get() = Rotation2d(2 * Math.PI / (2048 * chassisConstants.turnMotorGearRat) * (turnMotor.selectedSensorPosition % (2048 * chassisConstants.turnMotorGearRat)))

    fun falconToDegrees(counts: Double, gearRatio: Double): Double {
        return counts * (360.0 / (gearRatio * 2048.0))
    }

    fun degreesToFalcons(degrees: Double, gearRatio: Double): Double {
        return degrees / (360.0 / (gearRatio * 2048.0))
    }

    fun falconToRPM(velocityCounts: Double, gearRatio: Double): Double {
        val motorRPM = velocityCounts * (600.0 / 2048.0)
        return motorRPM / gearRatio
    }

    fun RPMToFalcon(RPM: Double, gearRatio: Double): Double {
        val motorRPM = RPM * gearRatio
        return motorRPM * (2048.0 / 600.0)
    }

    fun falconToMPS(
        velocitycounts: Double,
        circumference: Double,
        gearRatio: Double,
    ): Double {
        val wheelRPM = falconToRPM(velocitycounts, gearRatio)
        return wheelRPM * circumference / 60
    }

    fun MPSToFalcons(
        velocity: Double,
        circumference: Double,
        gearRatio: Double,
    ): Double {
        val wheelRPM = velocity * 60 / circumference
        return RPMToFalcon(wheelRPM, gearRatio)
    }

    fun falconToMeters(count: Double, wheelCircumference: Double, gearRat: Double): Double {
        return count * (wheelCircumference / (gearRat * 2048))
    }

    val state: SwerveModuleState
        get() {
            val velocity = falconToMPS(
                driveMotor.selectedSensorVelocity,
                chassisConstants.circumference,
                chassisConstants.turnMotorGearRat
            )
            val angle = turnAngle
            return SwerveModuleState(velocity, angle)
        }
    val position: SwerveModulePosition
        get() = SwerveModulePosition(
            falconToMeters(
                driveMotor.selectedSensorPosition,
                chassisConstants.circumference,
                chassisConstants.driveMotorGearRat
            ), turnAngle
        )

    fun setDesiredState(desiredState: SwerveModuleState) {
        var kDesiredState = desiredState
        kDesiredState = SwerveModuleState.optimize(kDesiredState, turnAngle)
        val velocity = MPSToFalcons(
            kDesiredState.speedMetersPerSecond,
            chassisConstants.circumference,
            chassisConstants.driveMotorGearRat
        )
        //double turnOutput = turnPIDController.calculate(turnAngleRadians(), desiredState.angle.getRadians());
        driveMotor[ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward] =
            feedforward.calculate(kDesiredState.speedMetersPerSecond)
        val nearestDegree = Math.round(kDesiredState.angle.degrees)
        val setTurnValue = (2048 / 360 * nearestDegree).toDouble()
        val inputAngle = nearestDegree.toDouble()
        val setPoint = setTurnValue * chassisConstants.turnMotorGearRat
        turnMotor[ControlMode.Position] = setPoint
    }

    fun stop() {
        driveMotor[ControlMode.Velocity] = 0.0
        turnMotor[ControlMode.Position] = state.angle.degrees * chassisConstants.turnMotorGearRat
    }

    override fun periodic() {
    }
    /**
     * simulate motors and sensors
     * the turn encoder should be the same as the turn motor's encoder but with it's gear ratio and offset applied
     */
    override fun simulationPeriodic() {
        // formula for torque in electrical motors: T = Kt * I
        // where Kt is the motor's torque constant and I is the current
        // to calculate Kt, we need to know the motor's resistance and back-EMF (voltage at no load)
        // emf formula: emf = Kv * rpm
        // given:
        /// Free Speed (RPM) - 6380
        /// Free Current (A) - 1.5
        /// Maximum Power (W) - 783
        /// Stall Torque (N · m) - 4.69
        /// Stall Current (A) - 257
        // we can calculate the motor's resistance and back-EMF
        // the back emf is = 6380 * 0.1047 = 668.66
        // the resistance is = 12 / 1.5 = 8
        // Kt = 4.69 / 257 = 0.0183
        // so the falcon 500's torque constant is 0.0183

        // simulate the drive motor
        driveMotor.run {
            val motorTorque = statorCurrent * 0.0183
            // get rps from the encoder
            val radiansPerSecond = (selectedSensorVelocity / 2048.0) * 2 * Math.PI

            // calculate acceleration of the rotor from torque and mass (wheel is 0.5 kg, gears are 0.1 kg)
            val acceleration = motorTorque / (0.5 + 0.1)
            // calculate the new velocity of the rotor
            val newVelocity = radiansPerSecond + acceleration * 0.02
            // calculate the new position of the rotor
            val newPosition = selectedSensorPosition + newVelocity * 0.02 * 2048.0 / (2 * Math.PI)

            // convert to falcons
            simCollection.setIntegratedSensorRawPosition(newPosition.toInt())
            simCollection.setIntegratedSensorVelocity(newVelocity.toInt())
        }

        // simulate the turn motor
        turnMotor.run {
            val motorTorque = statorCurrent * 0.0183
            // get rps from the encoder
            val radiansPerSecond = (selectedSensorVelocity / 2048.0) * 2 * Math.PI

            // calculate acceleration of the rotor from torque and mass (wheel is 0.5 kg, gears are 0.1 kg)
            val acceleration = motorTorque / (0.5 + 0.1)
            // calculate the new velocity of the rotor
            val newVelocity = radiansPerSecond + acceleration * 0.02
            // calculate the new position of the rotor
            val newPosition = selectedSensorPosition + newVelocity * 0.02 * 2048.0 / (2 * Math.PI)

            // convert to falcons
            simCollection.setIntegratedSensorRawPosition(newPosition.toInt())
            simCollection.setIntegratedSensorVelocity(newVelocity.toInt())
        }
        // take the turn motor's encoder and apply the gear ratio and offset, then set the turn encoder to that value
        absoluteEncoder.run {
            val turnMotorPosition = turnMotor.selectedSensorPosition
            val turnMotorVelocity = turnMotor.selectedSensorVelocity
            val turnMotorRPS = turnMotorVelocity / 2048.0 * 2 * Math.PI
            val turnMotorRPM = turnMotorRPS * 60.0 / 2.0 / Math.PI
            val turnMotorGearRPM = turnMotorRPM * chassisConstants.turnMotorGearRat
            val turnMotorGearRPS = turnMotorGearRPM / 60.0 * 2.0 * Math.PI
            val turnMotorGearVelocity = turnMotorGearRPS * 2048.0 / (2.0 * Math.PI)
            val turnMotorGearPosition = turnMotorPosition * chassisConstants.turnMotorGearRat
            simCollection.setRawPosition(turnMotorGearPosition.toInt())
            simCollection.setVelocity(turnMotorGearVelocity.toInt())
        }
    }
}