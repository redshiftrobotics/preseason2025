package frc.robot.subsystems.drive.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.logging.LoggedTunableNumber;
import frc.robot.utility.logging.LoggedTunableNumberGroup;
import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

  // Tunable parameters
  private static final LoggedTunableNumberGroup group =
      new LoggedTunableNumberGroup("Drive/HeadingController");

  private static final LoggedTunableNumber Kp =
      group.build("kP", HEADING_CONTROLLER_CONSTANTS.Kp());
  private static final LoggedTunableNumber Kd =
      group.build("kD", HEADING_CONTROLLER_CONSTANTS.Kd());

  private static final LoggedTunableNumber toleranceDegrees =
      group.build("ToleranceDegrees", HEADING_CONTROLLER_CONSTANTS.toleranceDegrees());

  private final Drive drive;

  private final ProfiledPIDController headingControllerRadians;

  /**
   * Creates a new HeadingController object
   *
   * @param drive drivetrain of robot
   */
  public HeadingController(Drive drive) {
    this.drive = drive;

    headingControllerRadians =
        new ProfiledPIDController(
            Kp.get(),
            0,
            Kd.get(),
            new TrapezoidProfile.Constraints(
                DRIVE_CONFIG.maxAngularVelocity(), DRIVE_CONFIG.maxAngularAcceleration()),
            Constants.LOOP_PERIOD_SECONDS);
    headingControllerRadians.enableContinuousInput(-Math.PI, Math.PI);
    headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
  }

  /** Reset last position and rotation to prepare for new use */
  public void reset() {
    headingControllerRadians.reset(
        drive.getPose().getRotation().getRadians(), drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Set goal heading. Calculate will now give values to get to this heading.
   *
   * @param heading desired heading of chassis
   */
  public void setGoal(Rotation2d heading) {
    headingControllerRadians.setGoal(heading.getRadians());
  }

  /**
   * Get goal heading.
   *
   * @return desired heading of chassis
   */
  public Rotation2d getGoal() {
    return new Rotation2d(headingControllerRadians.getGoal().position);
  }

  /**
   * Get speed chassis needs to rotation at to reach heading goal
   *
   * @param goalHeadingRadians desired heading of chassis
   * @return rotation speed to reach heading goal, omega radians per second
   */
  public double calculate(Rotation2d goalHeadingRadians) {
    setGoal(goalHeadingRadians);
    return calculate();
  }

  /**
   * Get speed chassis needs to rotation at to reach heading goal
   *
   * @return rotation speed to reach heading goal, omega radians per second
   */
  public double calculate() {

    // Update profiled PID controller
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          headingControllerRadians.setPID(Kp.get(), 0, Kd.get());
        },
        Kp,
        Kd);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          headingControllerRadians.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
        },
        toleranceDegrees);

    // Calculate output
    double measurement = drive.getPose().getRotation().getRadians();
    double output = headingControllerRadians.calculate(measurement);

    Logger.recordOutput(
        "Drive/HeadingController/Goal", headingControllerRadians.getGoal().position);
    Logger.recordOutput("Drive/HeadingController/Output", output);
    Logger.recordOutput(
        "Drive/HeadingController/HeadingError", headingControllerRadians.getPositionError());
    Logger.recordOutput("Drive/HeadingController/AtGoal", headingControllerRadians.atGoal());

    return output;
  }

  /**
   * Get if the chassis heading is our goal heading
   *
   * @return true if the absolute value of the position error is less than tolerance
   */
  public boolean atGoal() {
    return headingControllerRadians.atGoal();
  }
}
