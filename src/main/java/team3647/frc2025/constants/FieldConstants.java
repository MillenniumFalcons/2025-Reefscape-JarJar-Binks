package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Centimeter;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();
    
    public static final List<Pose2d> blueReefSides = List.of(
        new Pose2d(5.10, 5.11, kDefaultRot),
        new Pose2d(5.73, 4.02, kDefaultRot),
        new Pose2d(5.11, 2.96, kDefaultRot),
        new Pose2d(3.87, 2.96, kDefaultRot),
        new Pose2d(3.25, 4.02, kDefaultRot),
        new Pose2d(3.87, 5.09, kDefaultRot)
    );

    public static final List<Pose2d> redReefSides = List.of(
        new Pose2d(13.69, 5.11, kDefaultRot),
        new Pose2d(14.3, 4.02, kDefaultRot),
        new Pose2d(13.69, 2.96, kDefaultRot),
        new Pose2d(12.45, 2.96, kDefaultRot),
        new Pose2d(11.83, 4.02, kDefaultRot),
        new Pose2d(12.45, 5.09, kDefaultRot)
    );

    public static final Pose2d blueProcessor = new Pose2d(6.35, 0.48, kDefaultRot);
    public static final Pose2d redProcessor = new Pose2d(11.522, 7.568, kDefaultRot);

    public static final List<Pose2d> blueSources = List.of(
        new Pose2d(1.237, 7.109, Rotation2d.fromRadians(2.219)), //based on the degree i saw in choreo
        new Pose2d(1.237, 0.998, Rotation2d.fromRadians(-2.198))
    );

    public static final List<Pose2d> redSources = List.of(
        new Pose2d(16.384, 7.109, Rotation2d.fromRadians(0.936)),
        new Pose2d(16.384, 0.998, Rotation2d.fromRadians(-0.951))
    );

	//andrew put the fries in the bag

	//A1 is the left branch on the closest side, ccw from there
	public enum ScoringPos{
		A1(new Pose2d()),
		A2(new Pose2d()),
		B1(new Pose2d()),
		B2(new Pose2d()),
		C1(new Pose2d()),
		C2(new Pose2d()),
		D1(new Pose2d()),
		D2(new Pose2d()),
		E1(new Pose2d()),
		E2(new Pose2d()),
		F1(new Pose2d()),
		F2(new Pose2d());

		Pose2d pose;
		ScoringPos(Pose2d pose){
			this.pose = pose;
		}
	}


	public static Distance kFieldLength = Centimeter.of(1755);
	public static Distance kFieldWidth = Centimeter.of(805);






}
