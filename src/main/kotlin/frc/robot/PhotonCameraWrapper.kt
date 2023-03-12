package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.RobotBase
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*

class PhotonCameraWrapper {
    var photonCamera = PhotonCamera(
        VisionConstants.cameraName
    )

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
        return if (photonCamera.latestResult.targets.size > 2) {
            photonPoseEstimator!!.update()
        } else if (photonCamera.latestResult.targets.size == 2) {
            if (photonCamera.latestResult.targets
                    .minOf { it.poseAmbiguity } < 0.5
            )
                photonPoseEstimator!!.update()
            else Optional.empty()
        } else if (photonCamera.latestResult.targets.size == 1) {
            if (photonCamera.latestResult.targets
                    .minOf { it.poseAmbiguity } < 0.2
            )
                photonPoseEstimator!!.update()
            else Optional.empty()
        } else Optional.empty()
    }
}