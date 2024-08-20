package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;

public class VisionIOPhoton implements VisionIO {
	private final PhotonCamera camera;
	private final PhotonPoseEstimator odometry;

	/**
	 * Implements PhotonVision camera
	 *
	 * @param name Name of the camera.
	 * @param pose Location of the camera on the robot
	 */
	public VisionIOPhoton(String name, Transform3d pose, AprilTagFieldLayout fieldLayout) {

		// https://github.com/FRC-Sonic-Squirrels/2024-Robot-Code/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
		camera = new PhotonCamera(name);

		camera.setDriverMode(false);
		camera.setLED(VisionLEDMode.kOff);

		odometry = new PhotonPoseEstimator(fieldLayout,
				PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, pose);
		odometry.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		// PhotonPipelineResult result = camera.getLatestResult();
	}

	@Override
	public String getName() {
		return camera.getName();
	}

	@Override
	public void setReferencePose(Pose2d pose) {
		odometry.setReferencePose(pose);
	}
}
