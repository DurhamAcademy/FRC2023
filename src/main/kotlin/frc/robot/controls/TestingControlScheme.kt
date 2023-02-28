package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.absoluteValue
import kotlin.math.sqrt

class TestingControlScheme(
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
    val testFrontLeft: Trigger // the left stick is pointing to the top left with a deadband
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top left
            return Trigger { x < 0.1 && y > 0.1 && sqrt(x * x + y * y) > 0.5 }
        }
    val testFrontRight: Trigger
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the top right
            return Trigger { x > 0.1 && y > 0.1 && sqrt(x * x + y * y) > 0.5 }
        }
    val testBackLeft: Trigger
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the bottom left
            return Trigger { x < 0.1 && y < 0.1 && sqrt(x * x + y * y) > 0.5 }
        }
    val testBackRight: Trigger
        get() {
            // deadband
            val x = xboxCon.leftX.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            val y = xboxCon.leftY.coerceIn(-1.0, 1.0).let { if (it.absoluteValue < 0.05) 0.0 else it }
            // check if the stick is pointing to the bottom right
            return Trigger { x > 0.1 && y < 0.1 && sqrt(x * x + y * y) > 0.5 }
        }
    val testPercent: Double // rocket league style (trigger is throttle, other trigger is brake/reverse)
        get() {
            val throttle = xboxCon.leftTriggerAxis.coerceIn(0.0, 1.0)
            val brake = xboxCon.leftTriggerAxis.coerceIn(0.0, 1.0)
            return throttle - brake
        }
    val elevatorTest: Trigger
        get() = xboxCon.rightBumper().and(testMode)

    override val testGoToAprilTag1: Trigger
        get() = xboxCon.y().and { false }

    override val testGoToZeroZero: Trigger
        get() = xboxCon.x().and { false }
    override val testArm90: Trigger
        get() = xboxCon.povLeft()
    override val testArm0: Trigger
        get() = xboxCon.povUp()
    override val testArmNeg90: Trigger
        get() = xboxCon.povRight()

    override val testWrist90: Trigger
        get() = xboxCon.rightBumper()
    override val testWrist0: Trigger
        get() = xboxCon.povDown()
    override val testWristNeg90: Trigger
        get() = xboxCon.leftBumper()

    override val openManipulator: Trigger
        get() = Trigger { false }

    override val closeManipulator: Trigger
        get() = Trigger { false }

    override val grabCone: Trigger
        get() = xboxCon.start()

    override val holdCone: Trigger
        get() = xboxCon.back()
}