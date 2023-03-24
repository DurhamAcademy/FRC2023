package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.commands.alltogether.IOLevel
import frc.robot.constants.Constants
import frc.robot.constants.arm
import frc.robot.constants.elevator
import frc.robot.constants.elevator.elevatorMotor.tolerance.positionTolerance
import frc.robot.utils.GamePiece
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos

class Elevator(
    val robotContainer: RobotContainer?,
    val armController: Arm
) : SubsystemBase() {
    var hasLimitBeenPressed = false
    val armLength = 1.047
    val limitSwitchPressed: Boolean
        get() = !limitSwitch.get()
    val elevatorMotor = WPI_TalonFX(
        elevator.elevatorMotor.ElevatorMotorId
    ).apply {
        configFactoryDefault()
        setNeutralMode(NeutralMode.Brake)
        inverted = elevator.elevatorMotor.inverted
        selectedSensorPosition = 0.0
        // set status frames
        setStatusFramePeriod(StatusFrame.Status_1_General, 10)
        setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20)
        setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 100)
        setStatusFramePeriod(StatusFrame.Status_6_Misc, 100)
        setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 100)
        setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255)
        setStatusFramePeriod(StatusFrame.Status_10_Targets, 255)
        setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255)
        setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255)
        setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255)
        setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255)
    }
    val motorPid = ProfiledPIDController(
        elevator.elevatorMotor.PID.kP,
        elevator.elevatorMotor.PID.kI,
        elevator.elevatorMotor.PID.kD,
        TrapezoidProfile.Constraints(
            elevator.elevatorMotor.PID.TrapezoidProfile.maxVelocity,
            elevator.elevatorMotor.PID.TrapezoidProfile.maxAcceleration
        )
    ).apply {
        setTolerance(
            elevator.elevatorMotor.tolerance.positionTolerance,
            elevator.elevatorMotor.tolerance.velocityTolerance
        )
    }
    val feedforward = ElevatorFeedforward(
        elevator.elevatorMotor.Feedforward.kS,
        elevator.elevatorMotor.Feedforward.kG,
        elevator.elevatorMotor.Feedforward.kV,
        elevator.elevatorMotor.Feedforward.kA
    )
    val limitSwitch = DigitalInput(
        elevator.limitSwitch.ElevatorLimitSwitchId
    )

    val elevatorSim = ElevatorSim(
        DCMotor.getFalcon500(1),
        elevator.elevatorMotor.gearRatio,
        elevator.carriageMass,
        elevator.sproketRadius,
        elevator.limits.bottomLimit,
        elevator.limits.topLimit,
        true
    )
    var offset: Double = 0.0
    var simOffset: Double =
        Math.random() * (elevator.limits.topLimit -
                elevator.limits.bottomLimit) * 5

    inline var height: Double
        get() = if (RobotBase.isSimulation())
            motorPid.setpoint.position// + simOffset + offset
        else
            (elevatorMotor.selectedSensorPosition *
                    elevator.encoderDistancePerPulse *
                    elevator.elevatorMotor.gearRatio *
                    elevator.sproketRadius * 2 * Math.PI) +
                    elevator.limits.bottomLimit +
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
                        elevator.encoderDistancePerPulse *
                        elevator.elevatorMotor.gearRatio *
                        elevator.sproketRadius * 2.0 * PI -
                        elevator.limits.bottomLimit
                motorPid.reset(height)
        }

    var setpoint: Double = elevator.limits.bottomLimit
        set(value) {
//            println("SETPOINT $value")
            field = value.coerceIn(
                elevator.limits.bottomLimit,
                elevator.limits.topLimit
            )
        }
    var previousVoltage = 0.0
    fun setMotorVoltage(voltage: Double) {
        if (RobotBase.isSimulation())
            elevatorSim.setInputVoltage(
                voltage.coerceIn(
                    -RoboRioSim.getVInVoltage(),
                    RoboRioSim.getVInVoltage()
                )
            )
        else
            if (previousVoltage != voltage) {
                elevatorMotor.setVoltage(voltage.coerceIn(-8.0, 8.0))
                previousVoltage = voltage
            }
    }

    private var lastLimitSwitch: Boolean = true // switch is normally closed, so it should start true

    var elevatorTab = Shuffleboard.getTab("elevator")
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


    private var lastVel = 0.0
    private var lastTime = 0.0
    val tab = Shuffleboard.getTab("elevator")
    val heightEntry = tab.add("Height", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(
            mapOf(
                "min" to elevator.limits.bottomLimit,
                "max" to elevator.limits.topLimit
            )
        )
        .entry.apply {
            // set min
            this.setDouble(elevator.limits.bottomLimit)
        }

    val currentHeightEntry = tab.add("Current Height", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(
            mapOf(
                "min" to elevator.limits.bottomLimit,
                "max" to elevator.limits.topLimit
            )
        )
        .entry
    var zeroElevator = false
    val overHeight
        get() = height > (inchesToMeters(76.0) - (arm.length * cos(armController.armPosition)))

    override fun periodic() {
        SmartDashboard.putNumber("elevator Setpoint", setpoint)
        SmartDashboard.putString("elevcmd", this.currentCommand?.name ?: "NONE")
        currentHeightEntry.setDouble(this.height)
        // set the setpoint to the height entry
//        if (Constants.fullDSControl)
//            setpoint = heightEntry.getDouble(elevator.limits.bottomLimit)
        // set motor voltage
        
        if (zeroElevator) {
            setMotorVoltage(
                -1.5
            )
        } else {
            setMotorVoltage(
                motorPid.calculate(
                    height,
                    TrapezoidProfile.State(
                        setpoint.coerceAtMost(
                            inchesToMeters(60.0) - (armLength * cos(armController.armPosition))
                        ).coerceAtLeast(
                            elevator.limits.bottomLimit
                        ),
                        0.0
                    ),
                    TrapezoidProfile.Constraints(
                        elevator.elevatorMotor.PID.TrapezoidProfile.maxVelocity,
                        (if(overHeight) 2 else 1) * elevator.elevatorMotor.PID.TrapezoidProfile.maxAcceleration
                    )
                ) + feedforward.calculate(
                    motorPid.setpoint.velocity,
                )
            )
        }
        
        lastVel = motorPid.goal.velocity
        lastTime = Timer.getFPGATimestamp()

        SmartDashboard.putBoolean("LIMIT", limitSwitch.get())

//        setMotorVoltage(voltageSetEntry.getDouble(0.0))

        // check if its in teleop
//        if (RobotController.getUserButton()) setMotorVoltage(0.0)
//        else setMotorVoltage(12.0.coerceAtMost(RoboRioSim.getVInVoltage()))
        if (limitSwitch.get() != lastLimitSwitch) {
            this.height = elevator.limits.bottomLimit
            println("LIMIT PRESSED: ${limitSwitch.get()}")
            hasLimitBeenPressed = true
        }
        lastLimitSwitch = limitSwitch.get()
    }

    override fun simulationPeriodic() {
        if (RobotBase.isSimulation()) {
            elevatorSim.update(0.02)
        }
        SmartDashboard.putNumber("elevator Height", height)
        SmartDashboard.putNumber("elevator Setpoint", setpoint)
        SmartDashboard.putNumber(
            "Current Draw",
            if (RobotBase.isSimulation()) elevatorSim.currentDrawAmps
            else elevatorMotor.statorCurrent
        )
        SmartDashboard.putBoolean(
            "Has hit bottom limit",
            elevatorSim.hasHitLowerLimit()
        )
        SmartDashboard.putBoolean(
            "Has hit top limit",
            elevatorSim.hasHitUpperLimit()
        )
        // just set the motor voltage to the control scheme's output
    }

    fun isAtPosition(ioLevel: IOLevel, asObject: GamePiece) =
        when (asObject) {
            GamePiece.cone -> (height - ioLevel.coneElevatorHeight).absoluteValue < positionTolerance / 2
            GamePiece.cube -> (height - ioLevel.cubeElevatorHeight).absoluteValue < positionTolerance / 2
            else -> motorPid.atGoal()
        }
}