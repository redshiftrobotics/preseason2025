package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.
 *
 * <p>
 * Length refers to the <i>x</i> direction (as described by wpilib) Width refers to the <i>y</i>
 * direction (as described by wpilib)
 */
public class FieldConstants {

	public static final double fieldLength = Units.inchesToMeters(651.25);
	public static final double fieldWidth = Units.inchesToMeters(315.5);

	public static final double aprilTagWidth = Units.inchesToMeters(6.50);
	public final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
}
