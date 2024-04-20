package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class TeleopDriveController {
  private static final LoggedDashboardNumber controllerDeadband = new LoggedDashboardNumber("TeleopDrive/Deadband",
      0.1);

  private static final LoggedDashboardNumber maxAngularVelocityScalar = new LoggedDashboardNumber(
      "TeleopDrive/maxAngularVelocityScalar", 0.65);

  private static ChassisSpeeds getRobotRelativeSpeeds(
      Drive drive,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {

    // Get raw linear velocity
    final Translation2d linearVelocity = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // get magnitude of linear velocity, apply deadband for noise reduction and then
    // square it for better ramp up and fine control
    final double magnitude = MathUtil.applyDeadband(linearVelocity.getNorm(), controllerDeadband.get());
    final double magnitudeSquared = magnitude * magnitude;

    // get the squared linear velocity
    final Translation2d squaredLinearVelocity = new Pose2d(new Translation2d(), linearVelocity.getAngle())
        .transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
        .getTranslation();

    // Get rotation speed with deadband, then square it while keeping sign
    final double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), controllerDeadband.get());
    final double omegaSquared = Math.copySign(omega * omega, omega);

    // convert percentage speeds (-1 to 1) to actual meters per sec speeds
    return new ChassisSpeeds(
        squaredLinearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        squaredLinearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omegaSquared * drive.getMaxAngularSpeedRadPerSec() * maxAngularVelocityScalar.get());
  }

  public static ChassisSpeeds getFieldRelativeSpeeds(
      Drive drive,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        getRobotRelativeSpeeds(drive, xSupplier, ySupplier, omegaSupplier),
        AllianceFlipUtil.apply(drive.getPose().getRotation()));
  }

  public static ChassisSpeeds getSpeeds(
      Drive drive,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier,
      boolean isRobotRelative) {
    return isRobotRelative ? getRobotRelativeSpeeds(drive, xSupplier, ySupplier, omegaSupplier)
        : getFieldRelativeSpeeds(drive, xSupplier, ySupplier, omegaSupplier);
  }
}
