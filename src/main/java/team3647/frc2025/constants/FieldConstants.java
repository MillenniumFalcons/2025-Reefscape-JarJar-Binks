package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static final List<Pose2d> blueReefSides =
            List.of(
                    new Pose2d(5.10, 5.11, Rotation2d.kZero), // e
                    new Pose2d(5.73, 4.02, new Rotation2d(Radians.of(Math.PI / 3.0))), // d
                    new Pose2d(5.11, 2.96, new Rotation2d(Radians.of((2.0 * Math.PI) / 3.0))), // c
                    new Pose2d(3.87, 2.96, new Rotation2d(Radians.of(Math.PI))), // b
                    new Pose2d(3.25, 4.02, new Rotation2d(Radians.of((-2.0 * Math.PI) / 3.0))), // a
                    new Pose2d(3.87, 5.09, new Rotation2d(Radians.of((-Math.PI) / 3.0)))); // f

    public static final List<Pose2d> redReefSides =
            List.of(
                    new Pose2d(12.45, 2.96, kDefaultRot), // E
                    new Pose2d(11.83, 4.02, kDefaultRot), // D
                    new Pose2d(12.45, 5.09, kDefaultRot), // C
                    new Pose2d(13.69, 5.11, kDefaultRot), // b
                    new Pose2d(14.3, 4.02, kDefaultRot), // A
                    new Pose2d(13.69, 2.96, kDefaultRot)); // F

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
        A1(new Pose2d(3.072, 4.185, new Rotation2d(0))),
        A2(new Pose2d(3.072, 3.853, new Rotation2d(0))),
        B1(new Pose2d(3.652, 2.892, new Rotation2d(Radians.of(Math.PI / 3.0)))),
        B2(new Pose2d(3.933, 2.726, new Rotation2d(Radians.of(Math.PI / 3.0)))),
        C1(
                new Pose2d(
                        5.046,
                        2.711,
                        new Rotation2d(Radians.of((2.0 * Math.PI) / 3.0)))),
        C2(
                new Pose2d(
                        5.328,
                        2.871,
                        new Rotation2d(Radians.of((2.0 * Math.PI) / 3.0)))),
        D1(new Pose2d(5.918, 3.828, new Rotation2d(Radians.of(Math.PI)))),
        D2(new Pose2d(5.918, 4.191, new Rotation2d(Radians.of(Math.PI)))),
        E1(
                new Pose2d(
                        5.336,
                        5.158,
                        new Rotation2d(Radians.of((-2.0 * Math.PI) / 3.0)))),
        E2(
                new Pose2d(
                        5.07,
                        5.355,
                        new Rotation2d(Radians.of((-2.0 * Math.PI) / 3.0)))),
        F1(
                new Pose2d(
                        3.935,
                        5.335,
                        new Rotation2d(Radians.of((-Math.PI) / 3.0)))),
        F2(
                new Pose2d(
                        3.652,
                        5.164,
                        new Rotation2d(Radians.of((-Math.PI) / 3.0)))),
        NONE(new Pose2d());

        public Pose2d pose;

        ScoringPos(Pose2d pose) {
            this.pose = pose;
        }
    }

    public static Distance kFieldLength = Centimeter.of(1755);
    public static double kFieldLengthM = kFieldLength.in(Meter);
    public static Distance kFieldWidth = Centimeter.of(805);
    public static double kFieldWidthM = kFieldWidth.in(Meter);
}
