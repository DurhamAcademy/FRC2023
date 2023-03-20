package frc.robot.utils

import edu.wpi.first.wpilibj.DriverStation
import frc.robot.constants.Field2dLayout
import java.security.InvalidParameterException

val DriverStation.Alliance.xMul
    get() = when (this) {
        DriverStation.Alliance.Red -> Field2dLayout.Axes.Red.fieldOffsetMultiplier
        DriverStation.Alliance.Blue -> Field2dLayout.Axes.Blue.fieldOffsetMultiplier
        DriverStation.Alliance.Invalid -> throw InvalidParameterException("Alliance must be valid")
    }