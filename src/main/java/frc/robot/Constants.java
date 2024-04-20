package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static RobotType robotType = RobotType.SIM_BOT;

  public static final boolean tuningMode = false;

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMP_BOT;
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
      allianceChooser.addDefaultOption("Blue", Alliance.Blue);
      allianceChooser.addOption("Red", Alliance.Red);
    } else {
      allianceChooser = null;
    }
  }

  public static Alliance getAlliance() {
    if (allianceChooser != null) {
      return allianceChooser.get();
    }
    return DriverStation.getAlliance().isPresent()
        ? DriverStation.getAlliance().get()
        : Alliance.Blue;
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

  public static boolean disableHAL = false;

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
