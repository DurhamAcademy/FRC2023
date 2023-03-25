package frc.robot.utils

import edu.wpi.first.math.util.Units.inchesToMeters

enum class Slider(val fieldYValue: Double) {
    close(6.0 - inchesToMeters(4.0)), far(7.3 + inchesToMeters(4.0))
}