package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult

class Camera(
    val camera: PhotonCamera,
) : SubsystemBase() {
    lateinit var result: PhotonPipelineResult
    override fun periodic() {
        // This method will be called once per scheduler run
        result = camera.getLatestResult()

    }