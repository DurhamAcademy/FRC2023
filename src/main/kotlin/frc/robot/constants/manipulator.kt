package frc.robot.constants

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.util.Color

object manipulator {
    const val motorId = 31
    const val leftSolenoidForward = 0
    const val leftSolenoidReverse = 1
    const val rightSolenoidForward = 3
    const val rightSolenoidReverse = 2
    const val manipulatorCurrentLimit = 20.0

    val wristToObj = Units.inchesToMeters(8.0)
    const val confidenceThreshold = 0.75

    object Colors {
        val purpleCube = Color(0.19, 0.07, 0.77)
        val yellowCone = Color(0.95, 0.77, 0.06)
    }

    object Speeds {
        const val intake = 0.5
        const val outtake = -0.5
        const val stop = 0.0
        const val idle = 0.1
    }
}