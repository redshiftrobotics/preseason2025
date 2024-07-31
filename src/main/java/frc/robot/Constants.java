package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utility.Alert;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	private static final RobotType robotType = RobotType.SIM_BOT;

	public static final double LOOP_PERIOD_SECONDS = LoggedRobot.defaultPeriodSecs; // 0.02

	public static final boolean TUNING_MODE = true;

	private static boolean invalidRobot = false;

	public static RobotType getRobot() {
		if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
			new Alert(
					"Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR)
					.set(!invalidRobot);
			invalidRobot = true;
			return RobotType.COMP_BOT;
		}
		return robotType;
	}

	public static Mode getMode() {
		return switch (robotType) {
			case DEV_BOT, COMP_BOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
			case SIM_BOT -> Mode.SIM;
		};
	}

	private static final LoggedDashboardChooser<Alliance> allianceChooser;

	static {
		if (getMode() == Mode.SIM) {
			allianceChooser = new LoggedDashboardChooser<>("Alliance");
			allianceChooser.addDefaultOption("Auto", null);
			allianceChooser.addOption("Blue", Alliance.Blue);
			allianceChooser.addOption("Red", Alliance.Red);
		} else {
			allianceChooser = null;
		}
	}

	/**
	 * Get the current alliance based on driver station If {@code allianceChooser} is defined and has
	 * a value then use that instead.
	 *
	 * <p>
	 * Default to blue
	 *
	 * @return Current alliance
	 */
	public static Alliance getAlliance() {
		if (allianceChooser != null) {
			final Alliance chosenAlliance = allianceChooser.get();
			if (chosenAlliance != null) {
				return chosenAlliance;
			}
		}

		final Optional<Alliance> alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() : Alliance.Blue;
	}

	public enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public enum RobotType {
		SIM_BOT,
		DEV_BOT,
		COMP_BOT
	}

	private static boolean disableHAL = false;

	public static void disableHAL() {
		disableHAL = true;
	}

	/** Checks whether the correct robot is selected when deploying. */
	public static void main(String... args) {
		if (robotType == RobotType.SIM_BOT) {
			System.err.println("Cannot deploy, invalid robot selected: " + robotType);
			System.exit(1);
		}
	}
}
