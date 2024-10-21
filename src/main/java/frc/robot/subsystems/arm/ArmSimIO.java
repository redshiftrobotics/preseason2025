package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmSimIO implements ArmIO {
  private final SingleJointedArmSim sim;
  private PIDController controller;

  private double appliedVoltage;

  public ArmSimIO() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ArmConstants.GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(
                ArmConstants.ARM_LENGTH_METERS, 10), // random, can be gotten from cad
            ArmConstants.ARM_LENGTH_METERS,
            ArmConstants.ARM_MIN_ANGLE.getRadians(),
            ArmConstants.ARM_MAX_ANGLE.getDegrees(),
            true,
            0);
    controller = new PIDController(0, 0, 0);

    sim.setState(0, 0);

    runVolts(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();

    inputs.absoluteEncoderPositionRads = inputs.positionRads;
    inputs.relativeEncoderPositionRads = inputs.positionRads;

    inputs.appliedVolts = new double[] {appliedVoltage};
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runSetpoint(double positionRad, double feedForward) {
    runVolts(controller.calculate(sim.getAngleRads(), positionRad) + feedForward);
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    ;
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0);
  }

  @Override
  public void configurePID(double Kp, double Ki, double Kd) {
    controller = new PIDController(Kp, Ki, Kd);
  }
}
