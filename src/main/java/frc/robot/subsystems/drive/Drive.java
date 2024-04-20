package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LocalADStarAK;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);

  private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);

  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR

  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();

  private SwerveModulePosition[] lastModulePositions;

  private Pose2d pose = new Pose2d();

  private SwerveDrivePoseEstimator poseEstimator;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    modules =
        new Module[] {
          new Module(flModuleIO, new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0)),
          new Module(frModuleIO, new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)),
          new Module(blModuleIO, new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0)),
          new Module(brModuleIO, new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0))
        };

    this.gyroIO = gyroIO;

    kinematics =
        new SwerveDriveKinematics(modulesMap(Module::getDistanceFromCenter, Translation2d[]::new));

    lastModulePositions =
        modulesMap((m) -> new SwerveModulePosition(), SwerveModulePosition[]::new);
    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, pose);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getWheelSpeeds()),
        this::setRobotSpeeds,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < modules.length; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    modulesMap(Module::updateInputs);
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    modulesMap(Module::periodic);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      stop();
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    }

    // Log current wheel speeds
    Logger.recordOutput("SwerveStates/Measured", getWheelSpeeds().states);

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      pose = poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void setRobotSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs);

    SwerveDriveWheelStates wheelSpeeds = kinematics.toWheelSpeeds(discreteSpeeds);

    setWheelSpeeds(wheelSpeeds);
  }

  public void setWheelSpeeds(SwerveDriveWheelStates speeds) {

    SwerveDriveKinematics.desaturateWheelSpeeds(speeds.states, MAX_LINEAR_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setSpeeds(speeds.states[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", speeds.states);
  }

  /** Stops the drive. */
  public void stop() {
    modulesMap(Module::stop);
    setRobotSpeeds(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopUsingBrakeArrangement() {
    Rotation2d[] headings =
        modulesMap((module) -> module.getDistanceFromCenter().getAngle(), Rotation2d[]::new);
    kinematics.resetHeadings(headings);
    setRobotSpeeds(new ChassisSpeeds());
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getWheelPositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /**
   * Get all the swerve module states
   *
   * @return a {@link SwerveDriveWheelStates} object which contains an array of all swerve module
   *     states
   */
  public SwerveDriveWheelStates getWheelSpeeds() {
    return new SwerveDriveWheelStates(modulesMap(Module::getSpeeds, SwerveModuleState[]::new));
  }

  /**
   * Get all the swerve module positions
   *
   * @return a {@link SwerveDriveWheelPositions} object which contains an array of all swerve module
   *     positions
   */
  public SwerveDriveWheelPositions getWheelPositions() {
    return new SwerveDriveWheelPositions(
        modulesMap(Module::getPosition, SwerveModulePosition[]::new));
  }

  /**
   * Utility method. Function to easily run a function on each swerve module
   *
   * @param func function to run on each swerve module, takes one argument and returns nothing,
   *     operates via side effects.
   */
  private void modulesMap(Consumer<Module> func) {
    Arrays.stream(modules).forEach(func);
  }

  /**
   * Utility method. Function to easily run a function on each swerve module and collect results to
   * array. Insures that we don't mix up order of swerve modules, as this could lead to hard-to-spot
   * bugs.
   *
   * @param <T> type that is returned by function and should be collected
   * @param func function that gets some data off each swerve module
   * @param arrayInitializer constructor function for array to collect results in, use T[]::new
   * @return array of results from func.
   */
  private <T> T[] modulesMap(
      Function<? super Module, ? extends T> func, IntFunction<T[]> arrayInitializer) {
    // Private method with template argument T. Returns array of type T.
    // Takes in function that accepts a SwerveModule or something higher on the
    // inheritance chain (For example: Object, SubsystemBase)
    // and returns something of type T or something lower on the inheritance chain.
    // (For example if T is Object: Integer)
    // Also takes a T array initializer (T[]::new)
    return Arrays.stream(modules).map(func).toArray(arrayInitializer);
  }
}
