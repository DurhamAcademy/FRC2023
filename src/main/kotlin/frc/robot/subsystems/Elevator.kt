package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
        Constants.Elevator.sproketRadius,
        Constants.Elevator.limits.bottomLimit,
        Constants.Elevator.limits.topLimit,
        true
    )

    val height: Double
        get() = if (RobotBase.isSimulation())
            elevatorSim.positionMeters
        else
            elevatorMotor.selectedSensorPosition *
                    Constants.Elevator.encoderDistancePerPulse *
                    Constants.Elevator.gearRatio +
                    Constants.Elevator.limits.bottomLimit

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

    fun setMotorVoltage(voltage: Double) {
        if (RobotBase.isSimulation())
            elevatorSim.setInputVoltage(voltage)
        else
            elevatorMotor.setVoltage(voltage)
    }

    override fun periodic() {
        // limits
        if (height < Constants.Elevator.limits.bottomLimit) {
            setpoint = Constants.Elevator.limits.bottomLimit
        }
        if (height > Constants.Elevator.limits.topLimit) {
            setpoint = Constants.Elevator.limits.topLimit
        }

        // set motor voltage
        if (RobotController.getUserButton()) setMotorVoltage(
            motorPid.calculate(height, 4.0)
        )
        else setMotorVoltage(0.0)
        // check if its in teleop
//        if (RobotController.getUserButton()) setMotorVoltage(0.0)
//        else setMotorVoltage(12.0.coerceAtMost(RoboRioSim.getVInVoltage()))
    }

    override fun simulationPeriodic() {
        if (RobotBase.isSimulation()) {
            elevatorSim.update(0.02)
        }
        SmartDashboard.putNumber("Elevator Height", height)
        SmartDashboard.putNumber("Elevator Setpoint", setpoint)
        SmartDashboard.putNumber(
            "Current Draw",
            if (RobotBase.isSimulation()) elevatorSim.currentDrawAmps else elevatorMotor.statorCurrent
        )
        SmartDashboard.putBoolean("Has hit bottom limit", elevatorSim.hasHitLowerLimit())
        SmartDashboard.putBoolean("Has hit top limit", elevatorSim.hasHitUpperLimit())
        // just set the motor voltage to the control scheme's output

    }
}