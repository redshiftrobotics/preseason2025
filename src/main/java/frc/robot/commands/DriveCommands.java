package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private final Drive drive;

  private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;

  public DriveCommands(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {

    this.drive = drive;

    xSupplier = x;
    ySupplier = y;
    omegaSupplier = omega;
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    Translation2d linear = new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    double omega = omegaSupplier.getAsDouble();
    omega = Math.copySign(omega * omega, omega);

    // Calculate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linear.getAngle())
            .transformBy(new Transform2d(Math.pow(linear.getNorm(), 2), 0.0, new Rotation2d()))
            .getTranslation();

    return new ChassisSpeeds(
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omega * drive.getMaxAngularSpeedRadPerSec());
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        getRobotRelativeSpeeds(), AllianceFlipUtil.apply(drive.getPose().getRotation()));
  }
}
