package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import team3647.frc2025.subsystems.Elevator;

public class VisionConstants {
	public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

	public static final String kCrossbarLLName = "xbar";
	public static final String backRightCamName = "backRight";

	public static final Distance kOriginCubeToRobotBottom = Inches.of(1.75);

	public static final Transform3d LLsideMount = new Transform3d(
		Inches.of(-7.921), 
		Inches.of(3.681), 
		Inches.of(27.097), 
		new Rotation3d(Degrees.of(0), 
		Degrees.of(-55.018), 
		Degrees.of(-23.08)));

	public static final Transform3d LLCrossMount = new Transform3d(
		Inches.of(.00001772), 
		Inches.of(-4.434), 
		Inches.of(34.295), 
		new Rotation3d(Degrees.of(0), 
		Degrees.of(-7.382), 
		Degrees.of(180)));

	public static final Transform3d FrontLeft = new Transform3d(
		Inches.of(-9.5), 
		Inches.of(11.5), 
		Inches.of(7),
		new Rotation3d(Degrees.of(0),
		Degrees.of(0), 
		Degrees.of(-58.384)));

	public static final Transform3d BackRight = new Transform3d(
		Inches.of(-9.290),
		Inches.of(10.461),
		Inches.of(7.993),
		new Rotation3d(
			Degrees.of(0),
			Degrees.of(35.65),
			Degrees.of(162)
		));

}
