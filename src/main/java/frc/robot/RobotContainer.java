package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.teleop.DriveAtSpeeds;
import frc.robot.commands.teleop.DriveForwardAtHeading;
import frc.robot.commands.teleop.TeleopAngleDrive;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMaxCANCoder;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.utility.Alert;
import frc.robot.utility.OverrideSwitch;
import frc.robot.utility.Alert.AlertType;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;

	// Controller
	private final CommandGenericHID driverController = new CommandXboxController(0);

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
				break;

			case DEV_BOT:
				// Real robot, instantiate hardware IO implementations
				drive = new Drive(
						new GyroIONavX(),
						new ModuleIOSparkMaxCANCoder(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.BACK_LEFT_MODULE_CONFIG),
						new ModuleIOSparkMaxCANCoder(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
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
		configureDriverControllerBindings();
		configureOperatorControllerBindings();
	}

	private void configureDriverControllerBindings() {
		if (driverController instanceof CommandXboxController) {
			CommandXboxController driverXbox = (CommandXboxController) driverController;

			final BooleanSupplier useFieldRelative = () -> true;

			/**
			 * DEFAULT CONTROL MODE: Left stick controls translation (forward, backward, left, right),
			 * while right stick controls rotation (counter clockwise, clockwise spin). POV allows for
			 * field relative heading control using PID
			 *
			 * <p>
			 * ANGLE CONTROL MODE: Left stick still controls translation (forward, backward, left,
			 * right), while right stick controls field relative heading using PID. POV acts like mini
			 * tank drive controller
			 */
			final BooleanSupplier useAngleControlMode = new OverrideSwitch(driverXbox.y(), false, drive);

			drive.setDefaultCommand(
					Commands.either(
							new TeleopDrive(
									drive,
									() -> -driverXbox.getLeftY(),
									() -> -driverXbox.getLeftX(),
									() -> -driverXbox.getRightX(),
									useFieldRelative),
							new TeleopAngleDrive(
									drive,
									() -> -driverXbox.getLeftY(),
									() -> -driverXbox.getLeftX(),
									() -> -driverXbox.getRightY(),
									() -> -driverXbox.getRightX(),
									useFieldRelative),
							useAngleControlMode));

			driverXbox.pov(0)
					.whileTrue(
							Commands.either(
									new DriveForwardAtHeading(drive, Rotation2d.fromDegrees(+0)),
									new DriveAtSpeeds(drive, new ChassisSpeeds(1, 0, 0)),
									useAngleControlMode));
			driverXbox.pov(180)
					.whileTrue(
							Commands.either(
									new DriveForwardAtHeading(drive, Rotation2d.fromDegrees(+180)),
									new DriveAtSpeeds(drive, new ChassisSpeeds(-1, 0, 0)),
									useAngleControlMode));
			driverXbox.pov(90)
					.whileTrue(
							Commands.either(
									new DriveForwardAtHeading(drive, Rotation2d.fromDegrees(-90)),
									new DriveAtSpeeds(drive, new ChassisSpeeds(0, 0, Units.degreesToRadians(-90))),
									useAngleControlMode));
			driverXbox.pov(270)
					.whileTrue(
							Commands.either(
									new DriveForwardAtHeading(drive, Rotation2d.fromDegrees(-270)),
									new DriveAtSpeeds(drive, new ChassisSpeeds(0, 0, Units.degreesToRadians(+90))),
									useAngleControlMode));

			driverXbox.x().onTrue(Commands.runOnce(drive::brakeArrangementStop, drive));

		} else if (driverController instanceof CommandJoystick) {
			CommandJoystick driverJoystick = (CommandJoystick) driverController;

			drive.setDefaultCommand(
					new TeleopDrive(
							drive,
							() -> -driverJoystick.getY(),
							() -> -driverJoystick.getX(),
							() -> -driverJoystick.getTwist(),
							() -> true));
		}
	}

	private void configureOperatorControllerBindings() {
		/* Put operator control bindings here */
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
