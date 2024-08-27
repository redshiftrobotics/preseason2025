package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

/** IO layer interface for april tag detection systems */
public interface CameraIO {
	@AutoLog
	public static class CameraIOInputs {

		Pose3d estimatedRobotPose = null;
		double timestampSecondsFPGA = -1;

		int[] tagsUsed = new int[] {};

		boolean connected = false;
	}

	/** Get name of io camera */
	public default String getCameraName() {
		return "Camera";
	}

	/** Set april tag field layout to use */
	public default void setAprilTagFieldLayout(AprilTagFieldLayout layout) {
	}

	/** Get april tag field layout being used */
	public default AprilTagFieldLayout getAprilTagFieldLayout() {
		return null;
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(CameraIOInputs inputs) {
	}
}
