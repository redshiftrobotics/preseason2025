package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Drive control command for driving robot with x, y and omega control */
public class TeleopDrive extends Command {
  private static final LoggedTunableNumber controllerDeadband =
      new LoggedTunableNumber("TeleopDrive/Default/Deadband", 0.2);

  private static final LoggedTunableNumber maxAngularVelocityScalar =
      new LoggedTunableNumber("TeleopDrive/Default/maxAngularVelocityScalar", 0.65);

  private final Drive drive;
  private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
  private final BooleanSupplier fieldRelativeSupplier;

  /**
   * Creates a new TeleopAngleDrive Command. Meant to be default command for drivetrain.
   *
   * @param drive drivetrain of robot
   * @param xSupplier double from controller joystick for X (forward) velocity
   * @param ySupplier double from controller joystick for Y (left) velocity
   * @param omegaSupplier double from controller joystick for omega (rotation) velocity
   * @param fieldRelativeSupplier supply true for field relative, false for robot relative
   */
  public TeleopDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier) {

    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.stopUsingForwardArrangement();
  }

  @Override
  public void execute() {
    drive.setRobotSpeeds(getDesiredSpeeds(), fieldRelativeSupplier.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private ChassisSpeeds getDesiredSpeeds() {
    // Get raw linear velocity vector
    final Translation2d linearVelocity =
        new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // get length of linear velocity vector, and apply deadband to it for noise reduction
    final double magnitude =
        MathUtil.applyDeadband(linearVelocity.getNorm(), controllerDeadband.get());
    // Squaring the magnitude of the vector makes for better ramp up and better fine control
    final double magnitudeSquared = magnitude * magnitude;

    // get a vector with the same angle as the base linear velocity vector but with the magnitude
    // squared
    final Translation2d squaredLinearVelocity =
        new Pose2d(new Translation2d(), linearVelocity.getAngle())
            .transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
            .getTranslation();

    // Get rotation speed, and apply deadband
    final double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), controllerDeadband.get());
    // Square the omega value, make sure to copy the sign over for direction
    final double omegaSquared = Math.copySign(omega * omega, omega);

    // convert percentage speeds to actual meters per sec speeds
    return new ChassisSpeeds(
        squaredLinearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        squaredLinearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omegaSquared * drive.getMaxAngularSpeedRadPerSec() * maxAngularVelocityScalar.get());
  }
}
