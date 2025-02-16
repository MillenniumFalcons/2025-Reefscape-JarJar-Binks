package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class PoseUtils {

    public static boolean inCircle(Pose2d pose, Pose2d center, double radius) {
        return Math.abs(pose.getTranslation().getDistance(center.getTranslation())) <= radius;
    }

    public static boolean inCircle(Pose2d pose, Pose2d center, Distance radius) {
        return inCircle(pose, center, radius.in(Meter));
    }

	
	public static boolean inRect(Translation2d point, double lowX, double highX, double lowY, double highY){
		return point.getX() > lowX && point.getX() < highX && point.getY() > lowY && point.getY() < highY;
	}
}
