package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
	@AutoLog
    class VisionIOInputs {
        public Pose3d pose;
        public double timestamp;
    }

	/** Updates the set of loggable inputs. */
	default void updateInputs(VisionIOInputs inputs) {
	}

	/** Updates stored reference pose. */
	default void setReferencePose(Pose2d pose) {
	}

	/** Returns the name of the camera. */
	public default String getName() {
		return "";
	}
}
