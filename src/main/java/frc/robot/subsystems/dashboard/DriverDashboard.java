package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.SpeedController;

public class DriverDashboard extends SubsystemBase {

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private DriverDashboard() {}

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Subsystem ---

  private Drive drive;
  private SpeedController speedController;

  public void setDrive(Drive drive) {
    SmartDashboard.putData("Drive Subsystem", drive);
    this.drive = drive;
  }

  public void setSpeedController(SpeedController speedController) {
    this.speedController = speedController;
  }

  @Override
  public void periodic() {

    if (drive != null) {
      Pose2d pose = drive.getPose();
      ChassisSpeeds speeds = drive.getRobotSpeeds();

      SmartDashboard.putNumber("Heading Degrees", -pose.getRotation().getDegrees());
      SmartDashboard.putNumber(
          "Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);
    }

    if (speedController != null) {
      SmartDashboard.putString("Speed Level", speedController.getCurrentSpeedLevel().name());
      SmartDashboard.putString(
          "Speed Transl",
          String.format(
              "%.2f%%", speedController.getCurrentSpeedLevel().getTranslationCoefficient() * 100));
      SmartDashboard.putString(
          "Speed Rot",
          String.format(
              "%.2f%%", speedController.getCurrentSpeedLevel().getRotationCoefficient() * 100));
    }
  }
}
