package frc.robot.commands.teleop.input;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import java.util.function.BooleanSupplier;

public class DriverInput extends SubsystemBase {
	private final Drive drive;

	private final DoubleSupplier xSupplier, ySupplier, xAngleSupplier, yAngleSupplier;
	private final BooleanSupplier fieldRelativeSupplier;

	public DriverInput(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier xAngleSupplier,
			DoubleSupplier yAngleSupplier,
			BooleanSupplier fieldRelativeSupplier) {
		this.drive = drive;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.xAngleSupplier = xAngleSupplier;
		this.yAngleSupplier = yAngleSupplier;
		this.fieldRelativeSupplier = fieldRelativeSupplier;
	}

	public Translation2d getTranslationMetersPerSecond() {
		return DriverInputUtil.getTranslationMetersPerSecond(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
				drive.getMaxLinearSpeedMetersPerSec());
	}

	public Rotation2d getOmegaRadiansPerSecond() {
		return DriverInputUtil.getOmegaRadiansPerSecond(yAngleSupplier.getAsDouble(),
				drive.getMaxAngularSpeedRadPerSec());
	}

	public Rotation2d getHeadingDirection() {
		return DriverInputUtil.getHeadingDirection(xAngleSupplier.getAsDouble(), yAngleSupplier.getAsDouble());
	}

	public boolean getFieldRelative() {
		return fieldRelativeSupplier.getAsBoolean();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Angle Degrees", drive.getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("Speed Meters/Second", drive.getPose().getTranslation().getNorm());
	}
}
