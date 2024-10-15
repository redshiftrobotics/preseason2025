package frc.robot.subsystems.arm;

public class ArmConstants {

  public static final boolean ARE_MOTORS_REVERSED = false;
  public static final int LEFT_MOTOR_ID = 5;
  public static final int RIGHT_MOTOR_ID = 19;
  public static final int RIGHT_ENCODER_ID = 6;

  public static final double ARM_TOLERANCE_DEGREES = 2.0;

  public static final double ELEVATION_PID_P = 13.6;
  public static final double ELEVATION_PID_I = 0.0;
  public static final double ELEVATION_PID_D = 0.0;

  public static final double MAXIMUM_ARM_DEGREES = 50.0;
  public static final double MINIMUM_ARM_DEGREES = -120.0;

  public static final double ARM_START_DEGREES = -38.0;
  public static final double ARM_STOW_DEGREES = -83.0;
  public static final double ARM_STOW_2_DEGREES = -100.0;
  public static final double ARM_AMP_SHOOTING_DEGREES = -24;
  public static final double ARM_SPEAKER_SHOOTING_DEGREES = -105.822;
  public static final double ARM_SPEAKER_SHOOTING_DEGREES_FAR = -105.822 + 10;
  public static final double ARM_INTAKE_DEGREES = -110.5;
}
