package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;

public class CameraIOPhotonVision implements CameraIO {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator photonPoseEstimator;

	public CameraIOPhotonVision(CameraConfig config) {

		// --- Setup Camera ---
		camera = new PhotonCamera(config.cameraName());

		camera.setDriverMode(false);
		camera.setLED(VisionLEDMode.kOff);

		// --- Setup Pose Estimator ---

		// MULTI_TAG_PNP_ON_COPROCESSOR:
		// https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#enabling-multitag

		// PhotonPoseEstimator:
		// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html#using-a-photonposeestimator

		photonPoseEstimator = new PhotonPoseEstimator(
				AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				camera,
				config.robotToCamera());

		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	@Override
	public void setAprilTagFieldLayout(AprilTagFieldLayout fieldTags) {
		photonPoseEstimator.setFieldTags(fieldTags);
	}

	@Override
	public AprilTagFieldLayout getAprilTagFieldLayout() {
		return photonPoseEstimator.getFieldTags();
	}

	@Override
	public void updateInputs(CameraIOInputs inputs) {
		// https://github.com/FRC-Sonic-Squirrels/2024-Robot-Code/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java#L42

		Optional<EstimatedRobotPose> estimatedRobotPoseOptional = photonPoseEstimator.update();

		if (estimatedRobotPoseOptional.isPresent()) {
			EstimatedRobotPose estimateRobotPose = estimatedRobotPoseOptional.get();

			inputs.timestampSecondsFPGA = estimateRobotPose.timestampSeconds;
			inputs.estimatedRobotPose = estimateRobotPose.estimatedPose;
			inputs.tagsUsed = estimateRobotPose.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
		}

		inputs.connected = camera.isConnected();
	}


	@Override
	public String getCameraName() {
		return camera.getName();
	}
}
