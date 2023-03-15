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

    open val testGoToAprilTag1: Trigger = Trigger { false }
    open val testGoToZeroZero: Trigger = Trigger { false }

    open val testArm90: Trigger = Trigger { false }
    open val testArm0: Trigger = Trigger { false }
    open val testArmNeg90: Trigger = Trigger { false }

    open val testWrist90: Trigger = Trigger { false }
    open val testWrist0: Trigger = Trigger { false }
    open val testWristNeg90: Trigger = Trigger { false }

    open val openManipulator: Trigger = Trigger { false }
    open val closeManipulator: Trigger = Trigger { false }
    open val toggleManipulator: Trigger = Trigger { false }
    open val grabCone: Trigger = Trigger { false }
    open val holdCone: Trigger = Trigger { false }

    open val intakeFront: Trigger = Trigger { false }

    open val idleConfiguration: Trigger = Trigger { false }
    open val placeLvl1: Trigger = Trigger { false }
    open val placeLvl2: Trigger = Trigger { false }
    open val placeLvl3: Trigger = Trigger { false }

    open val intake: Trigger = Trigger { false }
    open val outtake: Trigger = Trigger { false }

    open val lowIntake: Trigger = Trigger { false }
    open val highIntake: Trigger = Trigger { false }

    open val moveToClosestHPS: Trigger = Trigger { false }
    open val moveToClosestScoreStation: Trigger = Trigger { false }

    open val autoBalance: Trigger = Trigger { false }
}