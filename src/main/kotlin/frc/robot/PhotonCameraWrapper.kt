package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.RobotBase
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*

class PhotonCameraWrapper(
    cameraName: String = VisionConstants.cameraName,
    val tagCountRequirement: ULong = 64UL
) {
    var photonCamera = PhotonCamera(
        cameraName
    )

    var validPoseCount = 0UL

    val canTrustPose: Boolean
        get() = validPoseCount > tagCountRequirement

    val percentage: Double
        get() = validPoseCount.toDouble() / tagCountRequirement.toDouble()

    //if is simulation, don't use photon camera
    /**
     * The pose estimator for the camera. This is null if the robot is in
     * simulation. This is because the PhotonVision library is not supported in
     * simulation.
     * @see PhotonPoseEstimator
     * @see PhotonCamera
     * @author
     */
    var photonPoseEstimator = if (RobotBase.isSimulation())
        null
    else
        PhotonPoseEstimator(
            AprilTagFieldLayout("/home/lvuser/deploy/2023-chargedup.json"),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
            photonCamera,
            VisionConstants.robotToCam
        )

    /**
     * @param prevEstimatedRobotPose The previous estimated robot pose, or null
     * if there is no previous pose.
     * @return A pair of the fused camera observations to a single Pose2d on the
     * field, and the time of the observation. Assumes a planar field and the
     * robot is always firmly on the ground
     */
    fun getEstimatedGlobalPose(prevEstimatedRobotPose: Pose2d?): Optional<EstimatedRobotPose> {
        if (photonPoseEstimator == null) return Optional.empty()
        photonPoseEstimator!!.setReferencePose(prevEstimatedRobotPose)

        val targets = photonCamera.latestResult.targets
        val bestTarget: PhotonTrackedTarget? =
            if (photonCamera.latestResult.hasTargets()) photonCamera.latestResult.bestTarget else null

        return if (targets.size > 2) {
            update()
        } else if (targets.size == 2) {
            if (targets
                    .minOf { it.poseAmbiguity } < 0.5
            )
                update()
            else Optional.empty()
        } else if (targets.size == 1) {
            if ((bestTarget?.poseAmbiguity ?: 1.0) < 0.2)
                update()
            else Optional.empty()
        } else Optional.empty()
    }

    private fun update(): Optional<EstimatedRobotPose> {
        validPoseCount++
        return photonPoseEstimator!!.update()
    }
}