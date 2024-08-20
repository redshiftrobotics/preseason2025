package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleLimits;

/** Singleton to store global state of robot */
public class RobotState {

	// --- Singleton ---

	private static RobotState instance;

	private RobotState() {
	}

	public static RobotState getInstance() {
		if (instance == null)
			instance = new RobotState();
		return instance;
	}

	// --- State ---

	@AutoLogOutput
	public boolean flywheelAccelerating = false;

	public ModuleLimits getModuleLimits() {
		return flywheelAccelerating && !DriverStation.isAutonomousEnabled()
				? DriveConstants.MODULE_LIMITS_FLYWHEEL_SPIN_UP
				: DriveConstants.MODULE_LIMITS_FREE;
	}

	public double flywheelShootRPMCompensation = 0;
}
