package frc.robot.commands.drivetrain

fun findNearest180(
    currentAngle: Double,
): Double {
    // Find the nearest 180 degree angle. the input should be in degrees with a range of -180 to 180
    // the output will be in degrees with a range of -180 to 180 and will be the closest 180 degree angle to the input
    // for example, if the input is 0, the output will be 0, if the input is 179, the output will be 180, if the input
    // is -179, the output will be 180 etc.
    var nearest180 = currentAngle % 360
    println("nearest180 = $nearest180")
    nearest180 = if (nearest180 > 180) {
        nearest180 - 360
    } else if (nearest180 < -180) {
        nearest180 + 360
    } else {
        nearest180
    }
    return if (nearest180 < 90 && nearest180 > -90) 0.0
    else 180.0
}