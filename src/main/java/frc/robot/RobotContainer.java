package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.teleop.DriveForwardAtHeading;
import frc.robot.commands.teleop.DriveRobotRelativeSpeeds;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.commands.teleop.TeleopHeadingControlledDrive;
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

		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		// Set up SysId routines
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)",
				drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption(
				"Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// Set up named commands for path planner auto
		NamedCommands.registerCommand("StopWithX", Commands.runOnce(drive::stopUsingBrakeArrangement, drive));

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

			final BooleanSupplier useFieldRelative = new OverrideSwitch(driverXbox.y(), true, drive, "Field Relative");

			final BooleanSupplier useAngleControlMode = new OverrideSwitch(driverXbox.a(), true, drive, "Angle Driven");

			final DriverInput input = new DriverInput(
					drive,
					() -> -driverXbox.getLeftY(),
					() -> -driverXbox.getLeftX(),
					() -> -driverXbox.getRightY(),
					() -> -driverXbox.getRightX(),
					useFieldRelative);

			final TeleopDrive teleop = new TeleopDrive(drive, input);

			drive.setDefaultCommand(teleop);

			for (int pov : List.of(0, 90, 180, 270)) {
				Rotation2d angle = Rotation2d.fromDegrees(-pov);
				driverXbox.pov(pov).and(useAngleControlMode).whileTrue(
						new DriveRobotRelativeSpeeds(drive,
								new ChassisSpeeds(angle.getCos(), 0, angle.getSin() * Math.PI)));

				driverXbox.pov(pov).and(() -> !useAngleControlMode.getAsBoolean())
						.whileTrue(new DriveForwardAtHeading(drive, angle));

				driverXbox.pov(pov).and(() -> !useAngleControlMode.getAsBoolean())
						.onFalse(new TeleopHeadingControlledDrive(drive, input, () -> angle));
			}

			driverXbox.x().onTrue(
					Commands.runOnce(drive::stopUsingBrakeArrangement, drive).withName("stopUsingBrakeArrangement"));

			SmartDashboard.putData(drive);

		} else if (driverController instanceof CommandJoystick) {
			final CommandJoystick driverJoystick = (CommandJoystick) driverController;

			DriverInput input = new DriverInput(
					drive,
					() -> -driverJoystick.getY(),
					() -> -driverJoystick.getX(),
					() -> -driverJoystick.getTwist(),
					() -> 0,
					() -> true);

			drive.setDefaultCommand(
					new TeleopDrive(drive, input));
		}
	}

	private void configureOperatorControllerBindings() {
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
