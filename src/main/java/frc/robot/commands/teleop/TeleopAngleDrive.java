package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.utility.AllianceFlipUtil;
import frc.robot.utility.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Drive control command for driving robot with x, y control and absolute angle control */
public class TeleopAngleDrive extends TeleopDrive {
	private static final LoggedTunableNumber controllerAngleDeadband = new LoggedTunableNumber(
			"TeleopDrive/AngleDeadband", 0.75);

	private final HeadingController headingController;

	public TeleopAngleDrive(
			Drive drive,
			Supplier<Translation2d> movementStick,
			Supplier<Translation2d> angleStick,
			BooleanSupplier fieldRelativeSupplier) {
		super(drive, movementStick, angleStick, fieldRelativeSupplier);
		headingController = new HeadingController(drive);
	}

	@Override
	public void initialize() {
		super.initialize();

		headingController.reset();
		headingController.setGoal(getDrive().getPose().getRotation());
	}

	public double getOmegaRadiansPerSecond(Translation2d inputs) {
		// If the vector length is longer then our deadband update the heading controller
		if (inputs.getNorm() > controllerAngleDeadband.get()) {
			headingController.setGoal(AllianceFlipUtil.apply(inputs.getAngle()));
		}

		return headingController.calculate();
	}
}
