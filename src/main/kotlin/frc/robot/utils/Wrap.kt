package frc.robot.utils

fun Int.wrap(min: Int, max: Int): Int = (this - min) % (max - min) + min