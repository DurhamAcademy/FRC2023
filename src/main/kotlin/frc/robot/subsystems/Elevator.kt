package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
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
    val feedforward = ElevatorFeedforward(
        Constants.Elevator.elevatorMotor.Feedforward.kS,
        Constants.Elevator.elevatorMotor.Feedforward.kG,
        Constants.Elevator.elevatorMotor.Feedforward.kV,
        Constants.Elevator.elevatorMotor.Feedforward.kA
    )
    val limitSwitch = DigitalInput(0).apply {
        if (RobotBase.isSimulation()) {
//            this.setSimDevice(simLimitSwitch)
        }
    }

    val elevatorSim = ElevatorSim(
        DCMotor.getFalcon500(1),
        Constants.Elevator.elevatorMotor.gearRatio,
        Constants.Elevator.carriageMass,
        Constants.Elevator.sproketRadius,
        Constants.Elevator.limits.bottomLimit,
        Constants.Elevator.limits.topLimit,
        true
    )
    var offset: Double = 0.0
    var simOffset: Double =
        Math.random() * (Constants.Elevator.limits.topLimit - Constants.Elevator.limits.bottomLimit) * 5
    var height: Double
        get() = if (RobotBase.isSimulation())
            elevatorSim.positionMeters// + simOffset + offset
        else
            (elevatorMotor.selectedSensorPosition *
                    Constants.Elevator.encoderDistancePerPulse *
                    Constants.Elevator.elevatorMotor.gearRatio) +
                    Constants.Elevator.limits.bottomLimit +
                    offset
        set(value) {
            if (RobotBase.isSimulation())
            // get the current position and set the offset to the difference
            // between the current position and the new position excluding
            // the current offset
                simOffset = value - elevatorSim.positionMeters - offset
            else
            // get the current position and set the offset to the difference
            // between the current position and the new position excluding
            // the current offset
                offset = value - elevatorMotor.selectedSensorPosition *
                        Constants.Elevator.encoderDistancePerPulse *
                        Constants.Elevator.elevatorMotor.gearRatio -
                        Constants.Elevator.limits.bottomLimit
        }

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
            elevatorSim.setInputVoltage(voltage.coerceIn(-RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage()))
        else
            elevatorMotor.setVoltage(voltage)
    }


    var elevatorTab = Shuffleboard.getTab("Elevator")
    var voltageSetEntry = elevatorTab.add("Voltage Set", 0.0)
        .withWidget("Number Slider")
        .withProperties(
            mapOf(
                "min" to -6.0,
                "max" to 6.0
            )
        )
        .entry.apply {
            // set zero
            this.setDouble(0.0)
        }
    private var lastLimitSwitch: Boolean = false

    override fun periodic() {
        // limits
        if(limitSwitch.get()){
            this.height = (Constants.Elevator.limits.topLimit)
        }
        // set motor voltage
        setMotorVoltage(
            motorPid.calculate(
                height,
                motorPid.goal
            ) + feedforward.calculate(
                height,
                motorPid.goal.velocity
            )
        )
//        setMotorVoltage(voltageSetEntry.getDouble(0.0))

        // check if its in teleop
        //if (RobotController.getUserButton()) setMotorVoltage(0.0)
//        else setMotorVoltage(12.0.coerceAtMost(RoboRioSim.getVInVoltage()))
        if ((limitSwitch.get() != lastLimitSwitch) && this.height==Constants.Elevator.limits.topLimit) {
            this.height = Constants.Elevator.limitSwitch.offset
            //fixme: uncomment this
        }
        lastLimitSwitch = limitSwitch.get()
    }

    override fun simulationPeriodic() {
        if (RobotBase.isSimulation()) {
            elevatorSim.update(0.02)
            // simulate the limit switch
            if (elevatorSim.hasHitLowerLimit()) {
//                limitSwitchBoolean.set(true)
            } else {
//                limitSwitchBoolean.set(false)
            }
        }
        SmartDashboard.putNumber("Elevator Height", height)
        SmartDashboard.putNumber("Elevator Setpoint", setpoint)
        SmartDashboard.putNumber(
            "Current Draw",
            if (RobotBase.isSimulation()) elevatorSim.currentDrawAmps
            else elevatorMotor.statorCurrent
        )
        SmartDashboard.putBoolean("Has hit bottom limit", elevatorSim.hasHitLowerLimit())
        SmartDashboard.putBoolean("Has hit top limit", elevatorSim.hasHitUpperLimit())
        // just set the motor voltage to the control scheme's output
    }
}