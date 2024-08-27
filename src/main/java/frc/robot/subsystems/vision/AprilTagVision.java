package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;

public class AprilTagVision extends SubsystemBase {

	private final Camera[] cameras;

	public AprilTagVision(CameraIO... camerasIO) {
		this.cameras = Arrays.stream(camerasIO)
				.map(io -> new Camera(io, FieldConstants.APRIL_TAG_FIELD_LAYOUT))
				.toArray(Camera[]::new);
	}

	@Override
	public void periodic() {
		cameras().forEach(Camera::periodic);
	}

	public Stream<Camera> cameras() {
		return Arrays.stream(cameras);
	}

	@Override
	public String toString() {
		return String.format("%s(%s)", getClass().getName(),
				Arrays.stream(cameras).map(Camera::getCameraName).collect(Collectors.joining(", ")));
	}
}
