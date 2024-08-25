package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.MODULE_CONSTANTS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utility.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;

/**
 * An individual swerve module in a drivetrain. This class is above the IO layer and contains
 * functionality for using each module regardless of hardware specifics.
 */
public class Module {

	private static final LoggedTunableNumber driveFeedForwardKs = new LoggedTunableNumber("Drive/Module/DriveFfKs",
			MODULE_CONSTANTS.feedForwardKs());
	private static final LoggedTunableNumber driveFeedForwardKv = new LoggedTunableNumber("Drive/Module/DriveFfKv",
			MODULE_CONSTANTS.feedForwardKv());

	private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/Module/DriveKp",
			MODULE_CONSTANTS.driveKp());
	private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/Module/DriveKd",
			MODULE_CONSTANTS.driveKd());

	private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/Module/TurnKp",
			MODULE_CONSTANTS.turnKp());
	private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/Module/TurnKd",
			MODULE_CONSTANTS.turnKd());

	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final Translation2d distanceFromCenter;

	private SimpleMotorFeedforward driveFeedforward;
	private final PIDController driveFeedback;
	private final PIDController turnFeedback;

	private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
	private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	/**
	 * Create an individual swerve module in a drivetrain.
	 *
	 * @param io                 swerve module io implantation
	 * @param distanceFromCenter distance from center of drivetrain to physical center of swerve module
	 */
	public Module(ModuleIO io, Translation2d distanceFromCenter) {
		this.io = io;
		this.distanceFromCenter = distanceFromCenter;

		driveFeedforward = new SimpleMotorFeedforward(driveFeedForwardKs.get(), driveFeedForwardKv.get(), 0);

		driveFeedback = new PIDController(driveKp.get(), 0, driveKd.get());
		turnFeedback = new PIDController(turnKp.get(), 0, turnKd.get());

		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
		setBrakeMode(true);
	}

	/**
	 * Update inputs without running the rest of the periodic logic. This is useful since these
	 * updates need to be properly thread-locked.
	 */
	public void updateInputs() {

		int hc = hashCode();

		LoggedTunableNumber.ifChanged(
				hc,
				() -> driveFeedforward = new SimpleMotorFeedforward(driveFeedForwardKs.get(), driveFeedForwardKv.get(),
						0),
				driveFeedForwardKs, driveFeedForwardKv);
		LoggedTunableNumber.ifChanged(
				hc,
				() -> driveFeedback.setPID(driveKp.get(), 0, driveKd.get()),
				driveKp, driveKd);
		LoggedTunableNumber.ifChanged(
				hc,
				() -> turnFeedback.setPID(turnKp.get(), 0, turnKd.get()),
				turnKp, turnKd);

		io.updateInputs(inputs);
	}

	/**
	 * Run periodic of module. This uses our PID controllers to try and reach our angle and speed setpoints.
	 */
	public void periodic() {
		Logger.processInputs("Drive/" + toString(), inputs);

		// On first cycle, reset relative turn encoder
		// Wait until absolute angle is nonzero in case it wasn't initialized yet
		// Use absolute position to generate offset for relative positions since we can read relative runs faster
		if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
			turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
		}

		// Run closed loop turn control
		if (angleSetpoint != null) {
			io.setTurnVoltage(
					turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

			// Run closed loop drive control
			// Only allowed if closed loop turn control is running
			if (speedSetpoint != null) {
				// Scale velocity based on turn error

				// When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
				// towards the setpoint, its velocity should increase. This is achieved by
				// taking the component of the velocity in the direction of the setpoint.

				// https://cdn1.byjus.com/wp-content/uploads/2019/05/Cosine-Graph.png
				double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

				// Run drive controller
				double velocityRadPerSec = adjustSpeedSetpoint / DRIVE_CONFIG.wheelRadius();
				io.setDriveVoltage(
						driveFeedforward.calculate(velocityRadPerSec)
								+ driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
			}
		}

		// Calculate positions for odometry
		int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
		odometryPositions = new SwerveModulePosition[sampleCount];

		for (int i = 0; i < sampleCount; i++) {
			double positionMeters = inputs.odometryDrivePositionsRad[i] * DRIVE_CONFIG.wheelRadius();

			Rotation2d angle = inputs.odometryTurnPositions[i].plus(
					turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());

			odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
		}
	}

	// --- Speeds ---

	/** Runs the module with the specified setpoint state. */
	public void setSpeeds(SwerveModuleState state) {
		// Optimize state based on current angle
		// Controllers run in "periodic" when the setpoint is not null
		state = SwerveModuleState.optimize(state, getAngle());

		// Update setpoints, controllers run in "periodic"
		speedSetpoint = state.speedMetersPerSecond;
		angleSetpoint = state.angle;
	}

	/** Returns the module state (turn angle and drive velocity). */
	public SwerveModuleState getSpeeds() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Returns the setpoint module state (turn angle and drive velocity) */
	public Optional<SwerveModuleState> getDesiredSpeeds() {
		return angleSetpoint == null
				? Optional.empty()
				: Optional.of(new SwerveModuleState(
						speedSetpoint == null ? 0 : speedSetpoint,
						angleSetpoint));
	}

	// --- Position ---

	/** Returns the module position (turn angle and drive position). */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	// --- Characterization ---

	/** Runs the module with the specified voltage while controlling to zero degrees. */
	public void runCharacterization(double volts) {
		// Closed loop turn control
		angleSetpoint = new Rotation2d();

		// Open loop drive control
		io.setDriveVoltage(volts);
		speedSetpoint = null;
	}

	/** Returns the drive velocity in radians/sec. */
	public double getCharacterizationVelocity() {
		return inputs.driveVelocityRadPerSec;
	}

	// --- Odometry ---

	/** Returns the distance module is from center of robot */
	public Translation2d getDistanceFromCenter() {
		return distanceFromCenter;
	}

	/** Returns the module positions received this cycle. */
	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	/** Returns the timestamps of the samples received this cycle. */
	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

	/** Disables all outputs to motors. */
	public void stop() {
		io.setTurnVoltage(0.0);
		io.setDriveVoltage(0.0);

		// Disable closed loop control for turn and drive
		angleSetpoint = null;
		speedSetpoint = null;
	}

	/** Sets whether brake mode is enabled. */
	public void setBrakeMode(boolean enabled) {
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	// --- Position and Speed Component Getters ---

	/** Returns the current turn angle of the module. */
	private Rotation2d getAngle() {
		if (turnRelativeOffset == null) {
			return new Rotation2d();
		} else {
			return inputs.turnPosition.plus(turnRelativeOffset);
		}
	}

	/** Returns the current drive position of the module in meters. */
	private double getPositionMeters() {
		return inputs.drivePositionRad * DRIVE_CONFIG.wheelRadius();
	}

	/** Returns the current drive velocity of the module in meters per second. */
	private double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadPerSec * DRIVE_CONFIG.wheelRadius();
	}

	// --- To String ---

	@Override
	public String toString() {

		final String[] yPositions = { "Back", "Middle", "Front" };
		final String[] xPositions = { "Right", "Middle", "Left" };

		final int ySignum = (int) Math.signum(distanceFromCenter.getY());
		final int xSignum = (int) Math.signum(distanceFromCenter.getX());

		return xPositions[xSignum + 1] + yPositions[ySignum + 1] + "SwerveModule";
	}
}
