package frc.robot.commands

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.controls.ControlScheme

class TestableControlScheme : ControlScheme() {
    var lastForward = 0.0
    var lastStrafe = 0.0
    var lastRotation = 0.0

    override val forward: Double
        get() = lastForward
    override val driveTest: Trigger
        get() = Trigger { false }

    override val turnTest: Trigger
        get() = Trigger { false }
    override val testFrontLeft: Trigger
        get() = Trigger { false }
    override val testFrontRight: Trigger
        get() = Trigger { false }
    override val testBackLeft: Trigger
        get() = Trigger { false }
    override val testBackRight: Trigger
        get() = Trigger { false }
    override val testPercent: Double
        get() = 0.0
    override val strafe: Double
        get() = lastStrafe
    override val xbox: CommandXboxController
        get() = CommandXboxController(0)
    override val rotation: Double
        get() = lastRotation

    override val testGoToAprilTag1: Trigger
        get() = Trigger { false }

    override val testGoToZeroZero: Trigger
        get() = Trigger { false }
}