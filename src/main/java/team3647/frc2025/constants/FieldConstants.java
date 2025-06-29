package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team3647.frc2025.Util.AllianceFlip;

import java.util.List;

public class FieldConstants {
    public static Distance kFieldLength = Centimeter.of(1755);
    public static double kFieldLengthM = kFieldLength.in(Meter);
    public static Distance kFieldWidth = Centimeter.of(805);
    public static double kFieldWidthM = kFieldWidth.in(Meter);

    public static final Rotation2d kDefaultRot = new Rotation2d();
    public static final Translation2d kRobotToReefA1 = new Translation2d(Inches.of(22), Inches.of(-12.94/2.0));
    public static final Translation2d kRobotToReefA1Safe = new Translation2d(Inches.of(29), Inches.of(-12.94/2.0));

    public static final Translation2d kRobotToReefFaceA = new Translation2d(Inches.of(17), Inches.of(0));
    private static final Pose2d BlueReefAMidpt = new Pose2d(Inches.of(144), Centimeter.of(805.0/2.0), Rotation2d.k180deg);
    public static final Translation2d kBlueReefOrigin = new Translation2d(Inches.of(144.0 - 14.0 + 93.5/2), Centimeter.of(805.0/2.0));
    public static final Pose2d kBlueReefA1 = BlueReefAMidpt.transformBy(new Transform2d(
        kRobotToReefA1,
        Rotation2d.k180deg
    ));

    public static final Pose2d kBlueReefA1Safe = BlueReefAMidpt.transformBy(new Transform2d(
        kRobotToReefA1Safe,
        Rotation2d.k180deg
    ));

    public static final Pose2d kBlueReefA2 = BlueReefAMidpt.transformBy(new Transform2d(
        new Translation2d(
            kRobotToReefA1.getX(),
            kRobotToReefA1.getY() * -1
        ),
        Rotation2d.k180deg
    ));

    public static final Pose2d kBlueReefA2Safe = BlueReefAMidpt.transformBy(new Transform2d(
        new Translation2d(
            kRobotToReefA1Safe.getX(),
            kRobotToReefA1Safe.getY() * -1
        ),
        Rotation2d.k180deg
    ));
    public static final Pose2d kBlueReefFaceA = BlueReefAMidpt.transformBy(new Transform2d(
        kRobotToReefFaceA,
        Rotation2d.k180deg
    ));

    public static final List<Pose2d> blueReefSides =
            List.of(
                new Pose2d(kBlueReefFaceA.getTranslation().rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(4.0 * Math.PI / 3.0))), kDefaultRot), // e
                    new Pose2d(kBlueReefFaceA.getTranslation().rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI))), kDefaultRot), // d
                    new Pose2d(kBlueReefFaceA.getTranslation().rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(2.0 * Math.PI / 3.0))), kDefaultRot), // c
                    new Pose2d(kBlueReefFaceA.getTranslation().rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI / 3.0))), kDefaultRot), // b
                    kBlueReefFaceA,
                    new Pose2d(kBlueReefFaceA.getTranslation().rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI / 3.0))), kDefaultRot) // F
                );

    public static final List<Pose2d> redReefSides = 
        getRedFromBlue(blueReefSides); 

    public static List<Pose2d> getRedFromBlue(List<Pose2d> poses){
        return List.of(
            AllianceFlip.flip(blueReefSides.get(0), Alliance.Red),
            AllianceFlip.flip(blueReefSides.get(1), Alliance.Red),
            AllianceFlip.flip(blueReefSides.get(2), Alliance.Red),
            AllianceFlip.flip(blueReefSides.get(3), Alliance.Red),
            AllianceFlip.flip(blueReefSides.get(4), Alliance.Red),
            AllianceFlip.flip(blueReefSides.get(5), Alliance.Red)
        );
        
    }

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

        A1(kBlueReefA1, kBlueReefA1Safe),
        A2(kBlueReefA2, kBlueReefA2Safe),
        B1(kBlueReefA1.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI/3.0))), kBlueReefA1Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI/3.0)))),
        B2(kBlueReefA2.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI/3.0))), kBlueReefA2Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI/3.0)))),
        C1(kBlueReefA1.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(2.0 * Math.PI/3.0))), kBlueReefA1Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(2.0 * Math.PI/3.0)))),
        C2(kBlueReefA2.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(2.0 * Math.PI/3.0))),  kBlueReefA2Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(2.0 * Math.PI/3.0)))),
        D1(kBlueReefA1.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI))),  kBlueReefA1Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI)))),
        D2(kBlueReefA2.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI))),  kBlueReefA2Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(Math.PI)))),
        E1(kBlueReefA1.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(4.0 * Math.PI/3.0))),  kBlueReefA1Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(4.0* Math.PI/3.0)))),
        E2(kBlueReefA2.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(4.0 * Math.PI/3.0))),  kBlueReefA2Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(4.0* Math.PI/3.0)))),
        F1(kBlueReefA1.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI/3.0))),  kBlueReefA1Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0* Math.PI/3.0)))),
        F2(kBlueReefA2.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI/3.0))),  kBlueReefA2Safe.rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI/3.0)))),
        PreloadF2BRUH(kBlueReefA2.plus(new Transform2d(-0.03, 0.0, Rotation2d.kZero)).rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI/3.0))),  kBlueReefA2Safe.plus(new Transform2d(-0.3, 0, Rotation2d.kZero)).rotateAround(kBlueReefOrigin, new Rotation2d(Radians.of(5.0 * Math.PI/3.0)))),

        NONE(new Pose2d(), new Pose2d());

        public Pose2d pose;
        public Pose2d safePose;
        public static Pose2d[] allPoses = {
            A1.pose, 
            A2.pose,
            B1.pose,
            B2.pose,
            C1.pose,
            C2.pose,
            D1.pose,
            D2.pose,
            E1.pose,
            E2.pose,
            F1.pose,
            F2.pose,
            A1.safePose,
            A2.safePose,
            B1.safePose,
            B2.safePose,
            C1.safePose,
            C2.safePose,
            D1.safePose,
            D2.safePose,
            E1.safePose,
            E2.safePose,
            F1.safePose,
            F2.safePose,
            PreloadF2BRUH.pose,
            PreloadF2BRUH.safePose
        };

        ScoringPos(Pose2d pose, Pose2d safe) {
            this.pose = pose;
            this.safePose = safe;

        }
    }

}