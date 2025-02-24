package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import javax.xml.crypto.KeySelector.Purpose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import team3647.frc2025.subsystems.Elevator;

public class VisionConstants {
	public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

	public static final String kCrossbarLLName = "xbar";
	public static final String kIntakeLLName = "coral";
	public static final String backRightCamName = "backRight";
	public static final String frontRightCamName = "frontRight";
	public static final String frontLeftCamName = "frontLeft";



	public static final Transform3d LLsideMount = new Transform3d(
		Inches.of(-7.921), 
		Inches.of(3.681), 
		Inches.of(27.097), 
		new Rotation3d(Degrees.of(0), 
		Degrees.of(-55.018), 
		Degrees.of(-23.08)));

	public static final Transform3d LLCrossMount = new Transform3d(
		Inches.of(.00001772), 
		Inches.of(-4.795749), 
		Inches.of(34.295), 
		new Rotation3d(Degrees.of(0), 
		Degrees.of(-7.382), 
		Degrees.of(180)));

		

	public static final Transform3d FrontRight = new Transform3d(
		Inches.of(12.168), 
		Inches.of(-9.087701), 
		Inches.of(7),
		new Rotation3d(Degrees.of(0),
		Degrees.of(0), 
		Degrees.of(-58.384)).rotateBy(new Rotation3d(Rotation2d.fromDegrees(90))));

		public static final Transform3d FrontLeft = new Transform3d(
			Inches.of(12.168), 
			Inches.of(9.087701), 
			Inches.of(7),
			new Rotation3d(Degrees.of(0),
			Degrees.of(0), 
			Degrees.of(58.384)).rotateBy(new Rotation3d(Rotation2d.fromDegrees(-90))));

	public static final Transform3d BackRight = new Transform3d(
		Inches.of(-9.579),
		Inches.of(11.029),
		Inches.of(8.450885),
		new Rotation3d(
			Degrees.of(0),
			Degrees.of(35.65),
			Degrees.of(0-90-27)
		));

}
