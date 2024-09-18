package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  // --- Vision Config ---

  public record CameraConfig(String cameraName, Transform3d robotToCamera) {}

  public static final CameraConfig FRONT_CAMERA =
      new CameraConfig(
          "frontCam", new Transform3d(new Translation3d(0, 0, 0.1), new Rotation3d(0, 0, 0)));
}
