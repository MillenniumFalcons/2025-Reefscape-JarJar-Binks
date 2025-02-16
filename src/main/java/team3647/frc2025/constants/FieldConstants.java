package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static final List<Pose2d> blueReefSides =
            List.of(
                    new Pose2d(5.10, 5.11, kDefaultRot),
                    new Pose2d(5.73, 4.02, kDefaultRot),
                    new Pose2d(5.11, 2.96, kDefaultRot),
                    new Pose2d(3.87, 2.96, kDefaultRot),
                    new Pose2d(3.25, 4.02, kDefaultRot),
                    new Pose2d(3.87, 5.09, kDefaultRot));

    public static final List<Pose2d> redReefSides =
            List.of(
                    new Pose2d(13.69, 5.11, kDefaultRot),
                    new Pose2d(14.3, 4.02, kDefaultRot),
                    new Pose2d(13.69, 2.96, kDefaultRot),
                    new Pose2d(12.45, 2.96, kDefaultRot),
                    new Pose2d(11.83, 4.02, kDefaultRot),
                    new Pose2d(12.45, 5.09, kDefaultRot));

    public static final Pose2d blueProcessor = new Pose2d(6.35, 0.48, kDefaultRot);
    public static final Pose2d redProcessor = new Pose2d(11.522, 7.568, kDefaultRot);

    public static final List<Pose2d> blueSources =
            List.of(
                    new Pose2d(
                            1.237,
                            7.109,
                            Rotation2d.fromDegrees(126)), // based on the degree i saw in choreo
                    new Pose2d(1.237, 0.998, Rotation2d.fromDegrees(-126)));

    public static final List<Pose2d> redSources =
            List.of(
                    new Pose2d(16.384, 7.109, Rotation2d.fromRadians(0.936)),
                    new Pose2d(16.384, 0.998, Rotation2d.fromRadians(-0.951)));

    // A1 is the left branch on the closest side, cOUTERcLOCKwISE from there
    // A1? AI? ccw? ccp? deepseek? dont ask deepseek to do 8*8 and turn it into a date
    public enum ScoringPos {
        A1(new Pose2d(3.153, 4.192, new Rotation2d(0))),
        A2(new Pose2d(3.187, 3.861, new Rotation2d(0))),
        B1(new Pose2d(3.688, 2.974, new Rotation2d(Radians.of(Math.PI / 3.0)))),
        B2(new Pose2d(3.9799, 2.818, new Rotation2d(Radians.of(Math.PI / 3.0)))),
        C1(new Pose2d(4.999, 2.8158, new Rotation2d(Radians.of((2.0 * Math.PI) / 3.0)))),
        C2(new Pose2d(5.282, 2.9829, new Rotation2d(Radians.of((2.0 * Math.PI) / 3.0)))),
        D1(new Pose2d(5.8026, 3.858, new Rotation2d(Radians.of(Math.PI)))),
        D2(new Pose2d(5.790, 4.1887, new Rotation2d(Radians.of(Math.PI)))),
        E1(new Pose2d(5.2765, 5.06289, new Rotation2d(Radians.of((-2.0 * Math.PI) / 3.0)))),
        E2(new Pose2d(4.998, 5.2329, new Rotation2d(Radians.of((-2.0 * Math.PI) / 3.0)))),
        F1(new Pose2d(3.9867, 5.2277, new Rotation2d(Radians.of((-Math.PI) / 3.0)))),
        F2(new Pose2d(3.7016, 5.06289, new Rotation2d(Radians.of((-Math.PI) / 3.0)))),
        NONE(new Pose2d());

        public Pose2d pose;

        ScoringPos(Pose2d pose) {
            this.pose = pose;
        }
    }

    public static Distance kFieldLength = Centimeter.of(1755);
    public static Distance kFieldWidth = Centimeter.of(805);
}
