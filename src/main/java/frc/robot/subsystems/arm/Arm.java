package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
    io.setSetpoint(ArmConstants.ARM_START_DEGREES);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
  }

  public void setPosition(double degrees) {
    io.setSetpoint(degrees);
  }
}
