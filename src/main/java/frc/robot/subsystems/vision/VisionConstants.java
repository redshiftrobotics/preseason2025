package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // --- Vision Config ---

    public record CameraConfig(String cameraName, Transform3d robotToCamera) {}

    public static final CameraConfig FRONT_CAMERA = new CameraConfig("", new Transform3d());
}
