package frc.robot.controls

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

abstract class ControlScheme {
    abstract val xbox: CommandXboxController?

    open val speedMutiplier: Double
        get() = 0.0

    open val rotation: Double
        get() = 0.0
    open val strafe: Double
        get() = 0.0
    open val forward: Double
        get() = 0.0


    open val toggleManipulator: Trigger = Trigger { false }

    open val idleConfiguration: Trigger = Trigger { false }
    open val placeLvl1: Trigger = Trigger { false }
    open val placeLvl2: Trigger = Trigger { false }
    open val placeLvl3: Trigger = Trigger { false }
    open val intakeHPS: Trigger = Trigger { false }
    open val lowIntake: Trigger = Trigger { false }

    open val spinIntakeIn: Trigger = Trigger { false }
    open val spinIntakeOut: Trigger = Trigger { false }

    open val moveToClosestHPSAxis: Trigger = Trigger { false }
    open val moveToClosestScoreStationAxis: Trigger = Trigger { false }

    open val autoBalance: Trigger = Trigger { false }

    open val ledColor: Trigger = Trigger { false }

    open val increaseEncoderAngle: Trigger = Trigger { false }
    open val decreaseEncoderAngle: Trigger = Trigger { false }

    open val alignClosestConeL1: Trigger = Trigger { false }
    open val alignClosestConeL2: Trigger = Trigger { false }
    open val alignClosestConeL3: Trigger = Trigger { false }

    open val alignClosestHPS: Trigger = Trigger { false }

    open val selectGridUp: Trigger = Trigger { false }
    open val selectGridDown: Trigger = Trigger { false }
    open val selectGridLeft: Trigger = Trigger { false }
    open val selectGridRight: Trigger = Trigger { false }

    open val confirmGridSelection: Trigger = Trigger { false }

    open val throwObject: Trigger = Trigger { false }

    open val lockSwerveModulesCircle: Trigger = Trigger { false }

    open val snapTo180: Trigger = Trigger { false }

    open val intakeGroundIntake: Trigger = Trigger { false }
    open val intakeEject: Trigger = Trigger { false }
    open val shootToLTwo: Trigger = Trigger { false }
    open val shootToLThree: Trigger = Trigger { false }

}