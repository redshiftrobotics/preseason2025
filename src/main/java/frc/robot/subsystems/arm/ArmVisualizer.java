package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {

  private final String key;

  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;

  public ArmVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm =
        new MechanismLigament2d("arm", ArmConstants.ARM_LENGTH_METERS, 0, 6, new Color8Bit(color));
    root.append(arm);
  }

  public void update(double angleRads) {
    arm.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism);

    Pose3d pivot =
        new Pose3d(
            ArmConstants.ARM_ORIGIN.getX(),
            0.0,
            ArmConstants.ARM_ORIGIN.getY(),
            new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Arm/Mechanism3d/" + key, pivot);
  }
}
