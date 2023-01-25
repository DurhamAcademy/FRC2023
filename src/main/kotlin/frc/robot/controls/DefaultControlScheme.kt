package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sqrt

class DefaultControlScheme(
    val xboxController: CommandXboxController,
) : ControlScheme() {
    override val rotation: Double // added deadband
        get() = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
    override val strafe: Double
        get() = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
    override val forward: Double
        get() = xboxController.rightY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it.pow(3) }
    override val driveTest : Trigger // left bumper and is in test mode
        get() = xboxController.leftBumper().and( testMode )
    override val turnTest : Trigger
        get() = xboxController.rightBumper().and( testMode )
    override val testFrontLeft: Trigger // the left stick is pointing to the top left with a deadband
        get() {
            // deadband
            val x = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxController.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top left
            return Trigger { x < 0.1 && y > 0.1 && sqrt(x*x + y*y) > 0.5 }
        }
    override val testFrontRight: Trigger
        get() {
            // deadband
            val x = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxController.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top right
            return Trigger { x > 0.1 && y > 0.1 && sqrt(x*x + y*y) > 0.5 }
        }
    override val testBackLeft: Trigger
            get() {
                // deadband
                val x = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
                val y = xboxController.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
                // check if the stick is pointing to the bottom left
                return Trigger { x < 0.1 && y < 0.1 && sqrt(x*x + y*y) > 0.5 }
            }
    override val testBackRight: Trigger
        get() {
            // deadband
            val x = xboxController.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxController.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the bottom right
            return Trigger { x > 0.1 && y < 0.1 && sqrt(x*x + y*y) > 0.5 }
        }
    override val testPercent: Double // rocket league style (trigger is throttle, other trigger is brake/reverse)
        get() {
            val throttle = xboxController.leftTriggerAxis.coerceIn(0.0, 1.0)
            val brake = xboxController.leftTriggerAxis.coerceIn(0.0, 1.0)
            return throttle - brake
        }
}