package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

    public static final String kCrossbarLLName = "limelight-xbar";
    public static final String kIntakeLLName = "limelight-coral";
    public static final String backRightCamName = "backRight";
    public static final String frontRightCamName = "frontRight";
    public static final String frontLeftCamName = "frontLeft";

    public static final Transform3d LLsideMount =
            new Transform3d(
                    Inches.of(3.68096),
                    Inches.of(-7.92081),
                    Inches.of(27.09660),
                    new Rotation3d(Degrees.of(0), Degrees.of(-55.018), Degrees.of(-23.08)));

    public static final Transform3d LLCrossMount =
            new Transform3d(
                    Inches.of(.00018),
                    Inches.of(-4.43416),
                    Inches.of(34.29541),
                    new Rotation3d(Degrees.of(0), Degrees.of(-7.382), Degrees.of(180)));

    public static final Transform3d FrontRight =
            new Transform3d(
                    Inches.of(12.168),
                    Inches.of(-9.087701),
                    Inches.of(7),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(31.665959)));

    public static final Transform3d FrontLeft =
            new Transform3d(
                    Inches.of(12.168),
                    Inches.of(9.087701),
                    Inches.of(7),
                    new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-31.665959)));

    public static final Transform3d BackRight =
            new Transform3d(
                    Inches.of(-9.28962),
                    Inches.of(10.46079),
                    Inches.of(7.99314),
                    new Rotation3d(Degrees.of(0), Degrees.of(35.65), Degrees.of(0 - 90 - 21.648)));

    public static final Transform3d BackLeft =
            new Transform3d( // this the most scuffed camera so expect to change these
                    Inches.of(-2.87203),
                    Inches.of(-8.35946),
                    Inches.of(28.72351),
                    new Rotation3d(Degrees.of(11.797), Degrees.of(-10), Degrees.of(65)));
}
