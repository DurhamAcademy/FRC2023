package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import frc.kyberlib.sensors.Limelight as KLimelight

class Limelight: SubsystemBase() {
    val limelight = KLimelight("gloworm")
    val camera = PhotonCamera("gloworm")

    fun setLEDMode(mode: KLimelight.LedMode) {
        limelight.ledMode = mode
    }

    fun setDriverMode(mode: Boolean) {
        limelight.driverMode = mode
    }
}