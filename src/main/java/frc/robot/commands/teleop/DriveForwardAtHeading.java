package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.HeadingController;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/** First rotates to specified angle, then starts slowing driving in that direction */
public class DriveForwardAtHeading extends Command {
  private static final LoggedDashboardNumber driveSpeed =
      new LoggedDashboardNumber("TeleopDrive/POV/driveSpeed", 1);

  private final Drive drive;
  private final HeadingController headingController;

  /**
   * Creates new DriveForwardAtAngle Command
   *
   * @param drive drivetrain of robot
   * @param heading field relative rotation to drive to aim at before driving forward
   */
  public DriveForwardAtHeading(Drive drive, Rotation2d heading) {
    this.drive = drive;

    headingController = new HeadingController(drive);
    headingController.setGoal(heading);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    headingController.reset();
  }

  @Override
  public void execute() {
    drive.setRobotSpeeds(getDesiredSpeeds(), false);
  }

  private ChassisSpeeds getDesiredSpeeds() {
    return new ChassisSpeeds(
        headingController.atGoal() ? driveSpeed.get() : 0, 0, headingController.calculate());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
