package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utility.logging.Alert;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private Constants() {}

  public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

  public static final boolean TUNING_MODE = false;

  private static RobotType robotType = RobotType.SIM_BOT;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.CANNON_BOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case CANNON_BOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_BOT -> Mode.SIM;
    };
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
    CANNON_BOT
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIM_BOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }
}
