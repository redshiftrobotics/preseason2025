package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.FEED_FORWARD_CONFIG;
import static frc.robot.subsystems.arm.ArmConstants.PID_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private final ArmFeedforward ffModel;

  private Goal goal = Goal.STOW;

  public enum ProfileConstraints {
    MAX(ArmConstants.PROFILED_CONSTRAINTS),
    SMOOTH(
        new TrapezoidProfile.Constraints(
            ArmConstants.PROFILED_CONSTRAINTS.maxVelocity * 0.75,
            ArmConstants.PROFILED_CONSTRAINTS.maxAcceleration * 0.5));

    private final TrapezoidProfile.Constraints constraints;

    private ProfileConstraints(TrapezoidProfile.Constraints constraints) {
      this.constraints = constraints;
    }

    private TrapezoidProfile.Constraints getConstraints() {
      return constraints;
    }
  }

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  public enum Goal {
    STOW(() -> Rotation2d.fromDegrees(-100.0)),
    UP(() -> Rotation2d.fromDegrees(-105.822));

    private final Supplier<Rotation2d> armSetpointSupplier;

    private Goal(Supplier<Rotation2d> armSetpointSupplier) {
      this.armSetpointSupplier = armSetpointSupplier;
    }

    private Rotation2d getAngle() {
      return armSetpointSupplier.get();
    }
  }

  /** Creates a new Flywheel. */
  public Arm(ArmIO io) {
    this.io = io;

    io.setBrakeMode(false);

    ffModel =
        new ArmFeedforward(
            FEED_FORWARD_CONFIG.Ks(),
            FEED_FORWARD_CONFIG.Kg(),
            FEED_FORWARD_CONFIG.Kv(),
            FEED_FORWARD_CONFIG.Ka());

    profile = new TrapezoidProfile(ProfileConstraints.SMOOTH.getConstraints());

    io.configurePID(PID_CONFIG.Kp(), PID_CONFIG.Ki(), PID_CONFIG.Kd());

    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public void setProfileConstraints(ProfileConstraints constraints) {
    profile = new TrapezoidProfile(constraints.getConstraints());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    io.periodic();

    setpointState =
        profile.calculate(
            Constants.LOOP_PERIOD_SECONDS,
            setpointState,
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    goal.getAngle().getRadians(),
                    ArmConstants.ARM_MIN_ANGLE.getRadians(),
                    ArmConstants.ARM_MAX_ANGLE.getRadians()),
                0.0));

    io.runSetpoint(
        setpointState.position, ffModel.calculate(setpointState.position, setpointState.velocity));

    goalVisualizer.update(goal.getAngle().getRadians());
    measuredVisualizer.update(inputs.positionRads);
    setpointVisualizer.update(setpointState.position);

    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
    Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Arm/Goal", goal);
  }
}
