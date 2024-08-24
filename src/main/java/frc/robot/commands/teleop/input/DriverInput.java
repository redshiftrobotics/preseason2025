package frc.robot.commands.teleop.input;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class DriverInput extends SubsystemBase {
	private final Drive drive;

	private final DoubleSupplier xSupplier, ySupplier, xAngleSupplier, yAngleSupplier;

	private SpeedLevel speedLevel;

	private final Field2d dashboardField2d = new Field2d();

	public static enum SpeedLevel {
		PRECISE(0.25, 0.1),
		DEFAULT(0.90, 0.60),
		BOOST(1, 0.75),
		MAX_BOOST(1, 1);

		public final double translationCoefficient, rotationCoefficient;

		private SpeedLevel(double translationCoefficient, double rotationCoefficient) {
			this.translationCoefficient = translationCoefficient;
			this.rotationCoefficient = rotationCoefficient;
		}
	}

	public DriverInput(
			Drive drive,
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier xAngleSupplier,
			DoubleSupplier yAngleSupplier) {
		this.drive = drive;
		this.xSupplier = xSupplier;
		this.ySupplier = ySupplier;
		this.xAngleSupplier = xAngleSupplier;
		this.yAngleSupplier = yAngleSupplier;

		this.speedLevel = SpeedLevel.DEFAULT;

		SmartDashboard.putData("Drive Subsystem", this.drive);
		// SmartDashboard.putData("Field2d", this.dashboardField2d);
	}

	public void setSpeedLevel(SpeedLevel speedLevel) {
		this.speedLevel = speedLevel;
	}

	public void increaseSpeedLevel() {
		this.speedLevel = SpeedLevel.values()[Math.min(speedLevel.ordinal() + 1, SpeedLevel.values().length - 1)];
	}

	public void decreaseSpeedLevel() {
		this.speedLevel = SpeedLevel.values()[Math.max(speedLevel.ordinal() - 1, 0)];
	}

	public SpeedLevel getSpeedLevel() {
		return speedLevel;
	}

	public Translation2d getTranslationMetersPerSecond() {
		return DriverInputUtil.getTranslationMetersPerSecond(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
				drive.getMaxLinearSpeedMetersPerSec() * this.speedLevel.translationCoefficient);
	}

	public Rotation2d getOmegaRadiansPerSecond() {
		return DriverInputUtil.getOmegaRadiansPerSecond(yAngleSupplier.getAsDouble(),
				drive.getMaxAngularSpeedRadPerSec() * this.speedLevel.translationCoefficient);
	}

	public Rotation2d getHeadingDirection() {
		return DriverInputUtil.getHeadingDirection(xAngleSupplier.getAsDouble(), yAngleSupplier.getAsDouble());
	}

	@Override
	public void periodic() {
		Pose2d pose = drive.getPose();

		dashboardField2d.setRobotPose(pose);

		ChassisSpeeds speeds = drive.getRobotSpeeds();
		SmartDashboard.putNumber("Heading Degrees", -pose.getRotation().getDegrees());
		SmartDashboard.putNumber("Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);

		SmartDashboard.putString("Speed Level", speedLevel.name());
		SmartDashboard.putString("Speed Transl", String.format("%.2f%%", speedLevel.translationCoefficient * 100));
		SmartDashboard.putString("Speed Rot", String.format("%.2f%%", speedLevel.rotationCoefficient * 100));
	}
}
