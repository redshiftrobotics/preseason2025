package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** This is the subsystem that represents the arm. */
public class ArmHardware implements ArmIO {

  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;

  private final CANcoder rightArmEncoder;

  private final StatusSignal<Double> armRotation;
  private final StatusSignal<Double> armRotationsPerSecond;

  private final PIDController armRaisePIDController;

  /** Constructor. Creates a new Arm Subsystem. */
  public ArmHardware(
      int leftMotorId, int rightMotorId, int rightEncoderId, boolean areMotorsReversed) {

    leftArmMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);

    rightArmEncoder = new CANcoder(rightEncoderId);

    armRaisePIDController =
        new PIDController(
            ArmConstants.ELEVATION_PID_P,
            ArmConstants.ELEVATION_PID_I,
            ArmConstants.ELEVATION_PID_D);
    armRaisePIDController.setTolerance(
        Units.degreesToRotations(ArmConstants.ARM_TOLERANCE_DEGREES));

    armRotation = rightArmEncoder.getAbsolutePosition();
    armRotationsPerSecond = rightArmEncoder.getVelocity();

    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    leftArmMotor.setInverted(areMotorsReversed);
    rightArmMotor.setInverted(!areMotorsReversed);

    leftArmMotor.enableVoltageCompensation(12.0);
    leftArmMotor.setSmartCurrentLimit(30);

    rightArmMotor.enableVoltageCompensation(12.0);
    rightArmMotor.setSmartCurrentLimit(30);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.appliedVolts = leftArmMotor.getAppliedOutput() * leftArmMotor.getBusVoltage();
    inputs.currentAmps = leftArmMotor.getOutputCurrent();

    inputs.rotation = armRotation.refresh().getValue();
    inputs.rotationPerSecond = armRotationsPerSecond.refresh().getValue();

    inputs.isAtDesiredPosition = armRaisePIDController.atSetpoint();
  }

  @Override
  public void setSetpoint(double degrees) {
    degrees =
        MathUtil.clamp(degrees, ArmConstants.MINIMUM_ARM_DEGREES, ArmConstants.MAXIMUM_ARM_DEGREES);

    SmartDashboard.putNumber("Arm SP Deg", degrees);

    armRaisePIDController.setSetpoint(Units.degreesToRotations(degrees));
  }

  @Override
  public void periodic() {
    double armRotations = armRotation.refresh().getValue();

    double armSpeed = armRaisePIDController.calculate(armRotations);

    leftArmMotor.set(armSpeed);
    rightArmMotor.set(armSpeed);

    SmartDashboard.putNumber("Arm Deg", Units.rotationsToDegrees(armRotations));
  }
}
