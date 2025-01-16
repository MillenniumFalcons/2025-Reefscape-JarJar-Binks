package team3647.frc2025.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();
    
    List<Pose2d> blueReefSides = List.of(
        new Pose2d(5.10, 5.11, kDefaultRot),
        new Pose2d(5.73, 4.02, kDefaultRot),
        new Pose2d(5.11, 2.96, kDefaultRot),
        new Pose2d(3.87, 2.96, kDefaultRot),
        new Pose2d(3.25, 4.02, kDefaultRot),
        new Pose2d(3.87, 5.09, kDefaultRot)
    );

    List<Pose2d> redReefSides = List.of(
        new Pose2d(13.69, 5.11, kDefaultRot),
        new Pose2d(14.3, 4.02, kDefaultRot),
        new Pose2d(13.69, 2.96, kDefaultRot),
        new Pose2d(12.45, 2.96, kDefaultRot),
        new Pose2d(11.83, 4.02, kDefaultRot),
        new Pose2d(12.45, 5.09, kDefaultRot)
    );





}
