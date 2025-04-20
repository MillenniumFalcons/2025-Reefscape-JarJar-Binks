package team3647.frc2025.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

public interface SwerveDrive extends PeriodicSubsystem, AllianceObserver {

    public void drive(double x, double y, double rotation);

    public void driveFieldOriented(double x, double y, double rotation);

    public void resetPose(Pose2d pose);

    public void setRobotPose(Pose2d pose);

    public Pose2d getOdoPose();

    public Orientation getPigeonOrientation();

    public void addVisionData(VisionMeasurement data);

    public boolean shouldAddData(VisionMeasurement data);
}
