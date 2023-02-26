package frc.robot.controls

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

abstract class ControlScheme {
    abstract val xbox: CommandXboxController?
    abstract val rotation: Double
    abstract val strafe: Double
    abstract val forward: Double
    open val testMode = Trigger { DriverStation.isTest() }
    abstract val driveTest: Trigger
    abstract val turnTest: Trigger
    abstract val testFrontLeft: Trigger
    abstract val testFrontRight: Trigger
    abstract val testBackLeft: Trigger
    abstract val testBackRight: Trigger
    abstract val testPercent: Double
    abstract val elevatorTest: Trigger

    abstract val testGoToAprilTag1: Trigger
    abstract val testGoToZeroZero: Trigger
}