@file:Suppress("UnusedImport")

package frc.robot

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

object ShuffleboardCommon {
    val mainTab: ShuffleboardTab = Shuffleboard.getTab("Main")
    val driveTab: ShuffleboardTab = Shuffleboard.getTab("Drive")
    val shooterTab: ShuffleboardTab = Shuffleboard.getTab("Debug")
}