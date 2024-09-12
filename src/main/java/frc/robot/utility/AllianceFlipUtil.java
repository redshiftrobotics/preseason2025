package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import java.util.Optional;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {

  private AllianceFlipUtil() {}

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) xCoordinate = FieldConstants.FIELD_LENGTH - xCoordinate;
    return xCoordinate;
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip())
      translation = new Translation2d(apply(translation.getX()), translation.getY());
    return translation;
  }

  /** Flips a rotation 180 degrees based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) rotation = new Rotation2d(-rotation.getCos(), rotation.getSin());
    return rotation;
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) pose = new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    return pose;
  }

  /** Flips a translation3d to the correct side of the field based on the current alliance color. */
  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip())
      translation3d =
          new Translation3d(
              apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    return translation3d;
  }

  /** Get whether to flip. If alliance is blue or unknown don't flip, if it is red then flip. */
  public static boolean shouldFlip() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }
}
