package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
	public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

	public static final String kBackLLName = "back";
	public static final String kPieceLLName = "piece";

	public static final String kBackRightPVName = "backRight";
	public static final String kBackLeftPVName = "backLeft";

	public static final Transform3d kRobotToFrontLeft = 
		new Transform3d(Inches.of(11.801), Inches.of(-9.146), Inches.of(7.158-1.625000), 
			new Rotation3d(Degree.of(0), Degree.of(0), Degree.of(-58.334)));
	public static final Transform3d kRobotToFrontRight = 
		new Transform3d(Inches.of(11.801), Inches.of(9.146), Inches.of(7.158-1.625000), 
			new Rotation3d(Degree.of(0), Degree.of(0), Degree.of(58.334)));
	
	
	
}
