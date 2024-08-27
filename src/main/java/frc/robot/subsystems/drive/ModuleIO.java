package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** IO layer interface for all swerve module hardware */
public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		double drivePositionRad = 0.0;
		double driveVelocityRadPerSec = 0.0;
		double driveAppliedVolts = 0.0;
		double[] driveCurrentAmps = new double[] {};

		Rotation2d turnAbsolutePosition = new Rotation2d();
		Rotation2d turnPosition = new Rotation2d();
		double turnVelocityRadPerSec = 0.0;
		double turnAppliedVolts = 0.0;
		double[] turnCurrentAmps = new double[] {};

		double[] odometryTimestamps = new double[] {};
		double[] odometryDrivePositionsRad = new double[] {};
		Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ModuleIOInputs inputs) {
	}

	/** Run the drive motor at the specified voltage. */
	public default void setDriveVoltage(double volts) {
	}

	/** Run the turn motor at the specified voltage. */
	public default void setTurnVoltage(double volts) {
	}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {
	}

	/** Enable or disable brake mode on the turn motor. */
	public default void setTurnBrakeMode(boolean enable) {
	}
}
