package team3647.frc2025.subsystems.Drivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import org.littletonrobotics.junction.AutoLog;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import team3647.frc2025.autos.AutoCommands.ChoreoController;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

public interface SwerveDrive extends PeriodicSubsystem, AllianceObserver {

    public void drive(double x, double y, double rotation);

    public void driveFieldOriented(double x, double y, double rotation);

    public void followTrajectory(SwerveSample sample, PIDController xController, PIDController yController, PIDController thetaController);
    public void followTrajectory(SwerveSample sample, ChoreoController controller);

    public void setRobotPose(Pose2d pose);

    public Pose2d getOdoPose();

    public Pose2d getRealPose();

    public Orientation getPigeonOrientation();

    public void addVisionData(VisionMeasurement data);

    public boolean shouldAddData(VisionMeasurement data);

    @AutoLog
    public class PeriodicIO {
        // inputs

        public boolean good = false;

        public double cachedVel = 0;
        public boolean isAccel = false;

        public double characterizationVoltage = 0;
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public Alliance color = Alliance.Blue;

        public SwerveModuleState[] states =
                new SwerveModuleState[] {
                    new SwerveModuleState(), // FL
                    new SwerveModuleState(), // FR
                    new SwerveModuleState(), // BL
                    new SwerveModuleState() // BR
                };

        public SwerveModuleState[] targets =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };

        public double timestamp = 0;

        public Pose2d visionPose = new Pose2d();

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

        // sim-specific stuff
        public ChassisSpeeds outputSpeeds = new ChassisSpeeds();
        public boolean fieldRelative = true;
        public Translation2d centerOfRotation = Translation2d.kZero;
        public Pose2d simPose = Pose2d.kZero;
        public ChassisSpeeds actualSpeeds = new ChassisSpeeds();
    }
}
