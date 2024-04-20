package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldConstants;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) xCoordinate = FieldConstants.fieldLength - xCoordinate;
    return xCoordinate;
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip())
      translation = new Translation2d(apply(translation.getX()), translation.getY());
    return translation;
  }

  /** Flips a rotation 180° based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) rotation = new Rotation2d(-rotation.getCos(), rotation.getSin());
    return rotation;
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) pose = new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    return pose;
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip())
      translation3d =
          new Translation3d(
              apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    return translation3d;
  }

  public static boolean shouldFlip() {
    return Constants.getAlliance() == Alliance.Red;
  }
}
