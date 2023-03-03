package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.manipulator.leftSolenoidForward
import frc.robot.Constants.manipulator.leftSolenoidReverse
import frc.robot.Constants.manipulator.motorId
import frc.robot.Constants.manipulator.rightSolenoidForward
import frc.robot.Constants.manipulator.rightSolenoidReverse
import frc.robot.GamePiece
import kotlin.math.pow
import frc.robot.Constants.manipulator as ManipConsts


class Manipulator: SubsystemBase() {

    private val motor = CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        setSmartCurrentLimit(ManipConsts.manipulatorCurrentLimit.toInt()) // add current limit to limit the torque
        setSecondaryCurrentLimit(20.0) // hard limit to prevent motor damage
    }
    private val leftsolenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, leftSolenoidForward, leftSolenoidReverse)
    private val rightsolenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, rightSolenoidForward, rightSolenoidReverse)

    var isOpen: Boolean?
        get() = when (leftsolenoid.get()) {
            Value.kForward -> false
            Value.kReverse -> true
            else -> null
        }
        set(value) {
            val v = when (value) {
                false -> Value.kForward
                true -> Value.kReverse
                else -> Value.kOff
            }
            leftsolenoid.set(v)
            rightsolenoid.set(v)
        }

    var motorPercentage: Double
        get() = motor.get()
        set(value) {
            motor.set(value)
        }


    private val i2cPort = I2C.Port.kOnboard
    val colorSensor = ColorSensorV3(i2cPort)


    val proximity: Int?
        get() = if (sensorConnected) colorSensor.proximity
        else null

    /**
     * the color detected by the sensor in the form of a Color object
     *
     * the color is null if the sensor is not connected or if the distance
     * to the object is greater than 6cm because the sensor works best when the
     * object is within 2in of the sensor
     * @see Color
     * @see ColorSensorV3.getColor
     * @return the color detected by the sensor
     */
    val color: Color?
        get() = if (sensorConnected)
            colorSensor.color
        else null


    private val distFilter = LinearFilter.singlePoleIIR(0.1, 0.02)

    /**
     * the distance from the sensor to the object in meters
     * @return the distance in meters
     *
     * the color sensor has a range of 0-10cm and returns a value of 0-2047
     * where 0 is the farthest and 2047 is the closest. this function converts
     * the value to cm with this formula:
     *
     * x=(10*(1-(y/2047))^(1/2))
     * - where y is the value returned by the sensor
     * - and x is the distance in cm
     *
     * the following equation does the same but returns the value in meters
     *
     * x=(0.1*(1-(y/2047))^(1/2))
     */
    var distance: Double? = null
    val sensorConnected: Boolean
        get() = colorSensor.isConnected

    val inColorRange: Boolean?
        get() = if (sensorConnected) distance!! < 0.15 else null

    private val colorMatch = ColorMatch().apply {
        addColorMatch(ManipConsts.Colors.purpleCube)
        addColorMatch(ManipConsts.Colors.yellowCone)
    }
    val objectType: GamePiece
        get() = if (sensorConnected) {
            if (inColorRange!!) {
                val color = colorSensor.color
                val match = colorMatch.matchColor(color)
                if (match == null || match.confidence < ManipConsts.confidenceThreshold)
                    GamePiece.unknown
                else when (match.color) {
                    ManipConsts.Colors.purpleCube -> GamePiece.cube
                    ManipConsts.Colors.yellowCone -> GamePiece.cone
                    else -> GamePiece.unknown
                }
            } else GamePiece.none
        } else GamePiece.none

    val tab = Shuffleboard.getTab("Manipulator")
    val motorCurrent = tab.add("Motor Current", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 40.0))
        .getEntry()
    val colorRed = tab.add("Red", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 1.0))
        .getEntry()
    val colorGreen = tab.add("Green", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 1.0))
        .getEntry()
    val colorBlue = tab.add("Blue", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 1.0))
        .getEntry()
    val colorIR = tab.add("IR", 0.0)
        .withWidget("Number Bar")
        .withProperties(mapOf("min" to 0.0, "max" to 1.0))
        .getEntry()


    override fun periodic() {
        if (sensorConnected) {
            val distFiltered = distFilter.calculate(colorSensor.proximity.toDouble())
            distance = 0.1 * (1 - (distFiltered / 2047.0)).pow(1 / 2.0)
            if (inColorRange!!) {
                val color = colorSensor.color

                SmartDashboard.putNumber("Red", color.red)
                SmartDashboard.putNumber("Green", color.green)
                SmartDashboard.putNumber("Blue", color.blue)
            }
            val IR = colorSensor.ir
            SmartDashboard.putNumber("IR", IR.toDouble())

            SmartDashboard.putNumber("Proximity", distance!!)
        } else {
            distance = null
        }

        motorCurrent.setDouble(motor.outputCurrent)
        colorRed.setDouble(color?.red ?: 0.0)
        colorGreen.setDouble(color?.green ?: 0.0)
        colorBlue.setDouble(color?.blue ?: 0.0)
        colorIR.setDouble(distance ?: 0.0)


    }
}