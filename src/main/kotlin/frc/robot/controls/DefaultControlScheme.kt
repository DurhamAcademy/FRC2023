package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.absoluteValue
import kotlin.math.sqrt

class DefaultControlScheme(
    override val xbox: CommandXboxController?
) : ControlScheme() {
     val xboxCon: CommandXboxController
        get() = xbox!!
    override val rotation: Double // added deadband
        get() = xboxCon.rightX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }

    // todo: add correct deadband from mathutils or kyberlib
    override val strafe: Double
        get() = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
    override val forward: Double
        get() = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
    override val driveTest : Trigger // left bumper and is in test mode
        get() = xboxCon.leftBumper().and( testMode )
    override val turnTest : Trigger
        get() = xboxCon.rightBumper().and( testMode )
    override val testFrontLeft: Trigger // the left stick is pointing to the top left with a deadband
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top left
            return Trigger { x < 0.1 && y > 0.1 && sqrt(x*x + y*y) > 0.5 }
        }
    override val testFrontRight: Trigger
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top right
            return Trigger { x > 0.1 && y > 0.1 && sqrt(x*x + y*y) > 0.5 }
        }
    override val testBackLeft: Trigger
            get() {
                // deadband
                val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
                val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
                // check if the stick is pointing to the bottom left
                return Trigger { x < 0.1 && y < 0.1 && sqrt(x*x + y*y) > 0.5 }
            }
    override val testBackRight: Trigger
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the bottom right
            return Trigger { x > 0.1 && y < 0.1 && sqrt(x * x + y * y) > 0.5 }
        }
    override val testPercent: Double // rocket league style (trigger is throttle, other trigger is brake/reverse)
        get() {
            val throttle = xboxCon.leftTriggerAxis.coerceIn(0.0, 1.0)
            val brake = xboxCon.leftTriggerAxis.coerceIn(0.0, 1.0)
            return throttle - brake
        }

    override val testGoToAprilTag1: Trigger
        get() = xboxCon.y()

    override val testGoToZeroZero: Trigger
        get() = xboxCon.x()
    override val testArm90: Trigger
        get() = xboxCon.b()
}