package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.teleop.input.DriverInput;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMaxCANCoder;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.examples.flywheel.Flywheel;
import frc.robot.subsystems.examples.flywheel.FlywheelIOSparkMax;
import frc.robot.utility.Alert;
import frc.robot.utility.Alert.AlertType;
import frc.robot.utility.OverrideSwitch;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final RobotState robotState = RobotState.getInstance();

	// Subsystems
	private final Drive drive;
	private final Flywheel flywheelExample;

	// Controller
	private final CommandGenericHID driverController = new CommandXboxController(0);
	private final CommandGenericHID operatorController = new CommandXboxController(1);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		switch (Constants.getRobot()) {
			case COMP_BOT:
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(
						new GyroIOPigeon2(true),
						new ModuleIOTalonFX(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
						new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
						new ModuleIOTalonFX(DriveConstants.BACK_LEFT_MODULE_CONFIG),
						new ModuleIOTalonFX(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
				flywheelExample = new Flywheel(new FlywheelIOSparkMax());
				break;

			case DEV_BOT:
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(
						new GyroIONavX(),
						new ModuleIOSparkMaxCANCoder(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.BACK_LEFT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
				flywheelExample = new Flywheel(new FlywheelIOSparkMax());
				break;

			case SIM_BOT:
				// Sim robot, instantiate physics sim IO implementations
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim(),
						new ModuleIOSim());
				flywheelExample = new Flywheel(new FlywheelIOSparkMax());
				break;

			default:
				// Replayed robot, disable IO implementations
				drive = new Drive(
						new GyroIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						},
						new ModuleIO() {
						});
				flywheelExample = new Flywheel(new FlywheelIOSparkMax());
				break;
		}

		// autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
		autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());

		// Set up SysId routines
		// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"Drive SysId (Dynamic Forward)",
				drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Dynamic Reverse)",
				drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Set up named commands for path planner auto
		// https://pathplanner.dev/pplib-named-commands.html
		NamedCommands.registerCommand("StopWithX", Commands.runOnce(drive::stopUsingBrakeArrangement, drive));

		// Path planner Autos
		// https://pathplanner.dev/gui-editing-paths-and-autos.html#autos
		autoChooser.addOption(
				"PathPlannerTest", new PathPlannerAuto("FirstTestAuto"));

		// Alerts for constants to avoid using them in competition
		if (Constants.TUNING_MODE) {
			new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
		}

		// Configure the button bindings
		configureControllerBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureControllerBindings() {
		CommandScheduler.getInstance().getActiveButtonLoop().clear();
		configureDriverControllerBindings();
		configureOperatorControllerBindings();
	}

	private void configureDriverControllerBindings() {
		if (driverController instanceof CommandXboxController) {
			final CommandXboxController driverXbox = (CommandXboxController) driverController;

			final Trigger useFieldRelative = new Trigger(
					new OverrideSwitch(driverXbox.y(), "Field Relative", OverrideSwitch.Mode.TOGGLE, true));

			final Trigger useAngleControlMode = new Trigger(
					new OverrideSwitch(driverXbox.a(), "Angle Driven", OverrideSwitch.Mode.HOLD, true));

			RobotModeTriggers.disabled().onTrue(drive.run(drive::stop));

			final DriverInput input = new DriverInput(
					drive,
					() -> -driverXbox.getLeftY(),
					() -> -driverXbox.getLeftX(),
					() -> -driverXbox.getRightY(),
					() -> -driverXbox.getRightX());

			drive.setDefaultCommand(drive.runEnd(() -> {
				Translation2d translation = input.getTranslationMetersPerSecond();
				Rotation2d rotation = input.getOmegaRadiansPerSecond();
				drive.setRobotSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians()),
						useFieldRelative.getAsBoolean());
			}, drive::stop).withName("DefaultDrive"));

			boolean includeDiagonalPOV = true;
			for (int pov = 0; pov < 360; pov += includeDiagonalPOV ? 45 : 90) {

				// POV angles are in Clock Wise Degrees, convert to standard Rotation
				final Rotation2d angle = Rotation2d.fromDegrees(-pov);

				// While the POV is being pressed and we are not in angle control mode,
				// set the chassis speeds to the Cos and Sin of the angle (so at 0 degrees forward by 1 left by 0, etc)
				driverXbox.pov(pov).and(useAngleControlMode.negate())
						.whileTrue(
								drive.startEnd(() -> drive.setRobotSpeeds(
										new ChassisSpeeds(angle.getCos(), angle.getSin(), 0)),
										drive::stop)
										.withName(String.format("DriveRobotRelative %s\u00B0", pov)));

				final HeadingController headingController = new HeadingController(drive, angle);

				driverXbox.pov(pov).and(useAngleControlMode)
						.whileTrue(
								Commands.runOnce(headingController::reset).andThen(drive.run(() -> {
									double rotationRadians = headingController.calculate();
									drive.setRobotSpeeds(new ChassisSpeeds(headingController.atGoal() ? 1 : 0, 0,
											headingController.atGoal() ? 0 : rotationRadians));
								}))
										.withName(String.format("ForwardLockedHeading %s\u00B0", pov)));

				driverXbox.pov(pov).and(useAngleControlMode)
						.onFalse(
								drive.run(() -> {
									Translation2d translation = input.getTranslationMetersPerSecond();
									double rotationRadians = headingController.calculate();
									drive.setRobotSpeeds(
											new ChassisSpeeds(translation.getX(), translation.getY(),
													headingController.atGoal() ? 0 : rotationRadians),
											useFieldRelative.getAsBoolean());
								}).until(() -> input.getOmegaRadiansPerSecond().getRadians() != 0)
										.withName(String.format("DriveLockedHeading %s\u00B0", pov)));
			}

			driverXbox.x().whileTrue(
					drive.startEnd(drive::stopUsingBrakeArrangement, drive::stopUsingForwardArrangement)
							.withName("StopWithX"));

			driverXbox.b().whileTrue(
					drive.runOnce(drive::stop)
							.andThen(Commands.idle(drive).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
									.withName("Cancel")));

			driverXbox.rightTrigger(0.2)
					.whileTrue(input.startEnd(input::increaseSpeedLevel, input::decreaseSpeedLevel));
			driverXbox.leftTrigger(0.2)
					.whileTrue(input.startEnd(input::decreaseSpeedLevel, input::increaseSpeedLevel));

		} else if (driverController instanceof CommandJoystick) {
			final CommandJoystick driverJoystick = (CommandJoystick) driverController;

			DriverInput input = new DriverInput(
					drive,
					() -> -driverJoystick.getY(),
					() -> -driverJoystick.getX(),
					() -> -driverJoystick.getTwist(),
					() -> 0);

			drive.setDefaultCommand(drive.startEnd(() -> {
				Translation2d translation = input.getTranslationMetersPerSecond();
				Rotation2d rotation = input.getOmegaRadiansPerSecond();
				drive.setRobotSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), rotation.getRadians()),
						true);
			}, drive::stop));
		}
	}

	private void configureOperatorControllerBindings() {
		if (Constants.getMode() == Constants.Mode.SIM && !DriverStation.isJoystickConnected(1)) {
			return;
		}

		if (operatorController instanceof CommandXboxController) {
			final CommandXboxController operatorXbox = (CommandXboxController) operatorController;

			// Adjust shot compensation
			operatorXbox
					.povUp()
					.whileTrue(
							Commands.runOnce(() -> robotState.flywheelShootRPMCompensation += 0.1)
									.andThen(Commands.waitSeconds(0.05))
									.ignoringDisable(true)
									.repeatedly());
			operatorXbox
					.povDown()
					.whileTrue(
							Commands.runOnce(() -> robotState.flywheelShootRPMCompensation -= 0.1)
									.andThen(Commands.waitSeconds(0.05))
									.ignoringDisable(true)
									.repeatedly());

			// Shoot
			operatorXbox.leftTrigger().whileTrue(
					Commands.startEnd(
							() -> flywheelExample.runVelocity(3000 + robotState.flywheelShootRPMCompensation),
							flywheelExample::stop,
							flywheelExample));
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
