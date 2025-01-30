package team3647.frc2025.Util;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseUtils {

    public static boolean inCircle(Pose2d pose, Pose2d center, double radius) {
        return Math.abs(pose.getTranslation().getDistance(center.getTranslation())) <= radius;
    }
}
