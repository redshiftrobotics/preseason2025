package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Map;

public class SwerveModuleOffsetReader extends Command {
  private static final String TAB_TITLE = "Swerve Module Offsets";

  private final Drive drive;
  private final ShuffleboardTab tab;
  private final GenericEntry speed;

  public SwerveModuleOffsetReader(Drive drive) {
    this.drive = drive;
    this.tab = Shuffleboard.getTab(TAB_TITLE);

    speed =
        tab.add("Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(3, 1)
            .withPosition(0, 0)
            .withProperties(
                Map.of(
                    "min",
                    -DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
                    "max",
                    DriveConstants.DRIVE_CONFIG.maxLinearVelocity()))
            .getEntry();

    tab.addNumber("Front Left", () -> drive.getWheelPositions().positions[0].angle.getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 2)
        .withSize(2, 2);

    tab.addNumber("Front Right", () -> drive.getWheelPositions().positions[1].angle.getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 2)
        .withSize(2, 2);

    tab.addNumber("Back Left", () -> drive.getWheelPositions().positions[2].angle.getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 4)
        .withSize(2, 2);

    tab.addNumber("Back Right", () -> drive.getWheelPositions().positions[3].angle.getRotations())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(4, 4)
        .withSize(2, 2);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.setMotorBrakeOnCoastModeEnabled(false);
    Shuffleboard.selectTab(TAB_TITLE);
  }

  @Override
  public void execute() {

    double driveSpeed = speed.getDouble(0);

    if (!RobotState.isDisabled()) {
      drive.setWheelSpeeds(
          new SwerveDriveWheelStates(
              new SwerveModuleState[] {
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(0)),
              }));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setMotorBrakeOnCoastModeEnabled(true);
    drive.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
