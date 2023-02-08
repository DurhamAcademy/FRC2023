package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.controls.ControlScheme

class Elevator(
    val controlScheme: ControlScheme,
) : SubsystemBase() {
    val elevatorMotor = WPI_TalonFX(Constants.Elevator.elevatorMotor.ElevatorMotorId)
    val motorPid = ProfiledPIDController(
        Constants.Elevator.elevatorMotor.PID.kP,
        Constants.Elevator.elevatorMotor.PID.kI,
        Constants.Elevator.elevatorMotor.PID.kD,
        TrapezoidProfile.Constraints(
            Constants.Elevator.elevatorMotor.PID.TrapezoidProfile.maxVelocity,
            Constants.Elevator.elevatorMotor.PID.TrapezoidProfile.maxAcceleration
        )
    )
    val motorFeedforward = ElevatorFeedforward(
        Constants.Elevator.elevatorMotor.FeedForeward.kS,
        Constants.Elevator.elevatorMotor.FeedForeward.kV,
        Constants.Elevator.elevatorMotor.FeedForeward.kA
    )

    val elevatorSim = ElevatorSim(
        DCMotor.getFalcon500(1),
        Constants.Elevator.elevatorMotor.gearRatio,
        Constants.Elevator.carriageMass,
        Constants.Elevator.sproketRadius
    )

    val height
        get() = elevatorMotor.selectedSensorPosition * Constants.Elevator.encoderDistancePerPulse * Constants.Elevator.gearRatio

    var setpoint: Double
        set(value) {
            if (value > Constants.Elevator.limits.topLimit)
                if (value < Constants.Elevator.limits.bottomLimit)
                    motorPid.goal = TrapezoidProfile.State(value, 0.0)
                else
                    motorPid.goal = TrapezoidProfile.State(Constants.Elevator.limits.bottomLimit, 0.0)
            else
                motorPid.goal = TrapezoidProfile.State(Constants.Elevator.limits.topLimit, 0.0)
        }
        get() = motorPid.goal.position

    override fun periodic() {
        // limits
        if (height < Constants.Elevator.limits.bottomLimit) {
            setpoint = Constants.Elevator.limits.bottomLimit
        }
        if (height > Constants.Elevator.limits.topLimit) {
            setpoint = Constants.Elevator.limits.topLimit
        }

        elevatorMotor.set(
            motorPid.calculate(height, motorPid.goal) +
                    motorFeedforward.calculate(height, motorPid.goal.velocity)
        )
    }

}