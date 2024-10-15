package frc.robot;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.utility.swerve254util.ModuleLimits;

/** Singleton to store global state of robot, try to use sparingly */
public class RobotState {

  // --- Singleton Setup ---

  private static RobotState instance;

  private RobotState() {}

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // --- State Example---

  public ModuleLimits getModuleLimits() {
    return DriveConstants.MODULE_LIMITS_FREE;
  }
}
