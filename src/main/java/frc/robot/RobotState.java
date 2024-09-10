package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

/** Singleton to store global state of robot, try to use sparingly */
public class RobotState {

	// --- Singleton Setup ---

	private static RobotState instance;

	private RobotState() {}

	public static RobotState getInstance() {
		if(instance == null) instance = new RobotState();
		return instance;
	}

	// --- State Example---

	@AutoLogOutput
	public boolean flywheelAccelerating = false;

	public frc.robot.utility.swerve254util.ModuleLimits getModuleLimits() {
		return flywheelAccelerating && !DriverStation.isAutonomousEnabled() ?
			DriveConstants.MODULE_LIMITS_FLYWHEEL_SPIN_UP :
			DriveConstants.MODULE_LIMITS_FREE;
	}

	public double flywheelShootRPMCompensation = 0;

	public void adjustFlywheelShotRPM(double rpm) {
		flywheelShootRPMCompensation += rpm;
	}
}
