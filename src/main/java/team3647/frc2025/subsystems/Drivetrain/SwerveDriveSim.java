package team3647.frc2025.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

public class SwerveDriveSim implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simSwerve;

    private PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();
    private final double kDt;

    public SwerveDriveSim(DriveTrainSimulationConfig config, double kDt) {
        if(RobotBase.isReal()) throw new IllegalStateException("TRYING TO RUN SIM DT ON REAL RIO");
        this.simSwerve =
                new SelfControlledSwerveDriveSimulation(
                        new SwerveDriveSimulation(
                                config, Pose2d.kZero.rotateBy(Rotation2d.k180deg)));
        this.kDt = kDt;
        SimulatedArena.getInstance().addDriveTrainSimulation(simSwerve.getDriveTrainSimulation());
    }

    @Override
    public void drive(double x, double y, double rotation) {
        periodicIO.outputSpeeds.vxMetersPerSecond = x;
        periodicIO.outputSpeeds.vyMetersPerSecond = y;
        periodicIO.outputSpeeds.omegaRadiansPerSecond = rotation;

        periodicIO.fieldRelative = false;
    }
    Pose2d previouspose = new Pose2d();

    @Override
    public void driveFieldOriented(double x, double y, double rotation) {
        periodicIO.outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, periodicIO.simPose.getRotation());


        periodicIO.fieldRelative = false;
    }

    public boolean shouldAddData(VisionMeasurement measurement) {
        double distance = measurement.pose.minus(getOdoPose()).getTranslation().getNorm();
        double angle =
                measurement.pose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        return (distance < MathUtil.clamp(getVelSquared(), 0.25, 1.5) && Math.abs(angle) < 15)
                || DriverStation.isAutonomous();
    }

    private double getVelSquared() {
        return periodicIO.measuredSpeeds.vxMetersPerSecond
                        * periodicIO.measuredSpeeds.vxMetersPerSecond
                + periodicIO.measuredSpeeds.vyMetersPerSecond
                        * periodicIO.measuredSpeeds.vyMetersPerSecond;
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
    }

    @Override
    public void writePeriodicOutputs() {
        simSwerve.periodic();
        simSwerve.runChassisSpeeds(
                periodicIO.outputSpeeds.times(1),    
                periodicIO.centerOfRotation,
                periodicIO.fieldRelative,
                true);


        
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.states[0] = simSwerve.getMeasuredStates()[0];
        periodicIO.states[1] = simSwerve.getMeasuredStates()[1];
        periodicIO.states[2] = simSwerve.getMeasuredStates()[2];
        periodicIO.states[3] = simSwerve.getMeasuredStates()[3];

        

        periodicIO.pose = simSwerve.getOdometryEstimatedPose();
        periodicIO.simPose = simSwerve.getActualPoseInSimulationWorld();
        periodicIO.measuredSpeeds = simSwerve.getMeasuredSpeedsFieldRelative(true);
        periodicIO.actualSpeeds = simSwerve.getActualSpeedsFieldRelative();
        periodicIO.timestamp = Timer.getFPGATimestamp();

        Logger.recordOutput("vel", periodicIO.simPose.minus(previouspose).getX()/(0.004));


        Logger.recordOutput("simSwerve.maxLinearVelocity", simSwerve.maxLinearVelocity().in(MetersPerSecond));

        // periodicIO.visionPose = //set when poton sim is done

        Logger.processInputs(getName(), periodicIO);

        previouspose = periodicIO.simPose;

    }
    @Override
    public void resetPose(Pose2d pose) {
        simSwerve.resetOdometry(pose);
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        simSwerve.resetOdometry(pose);
        simSwerve.setSimulationWorldPose(pose);
        periodicIO = new PeriodicIOAutoLogged();
    }

    @Override
    public Pose2d getOdoPose() {
        return periodicIO.pose;
    }

    @Override
    public Pose2d getRealPose() {
        return periodicIO.simPose;
    }

    public Pose2d getSimPose() {
        return periodicIO.simPose;
    }

    @Override
    public Orientation getPigeonOrientation() {
        return new Orientation(periodicIO.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public void onAllianceFound(Alliance color) {
        periodicIO.color = color;
    }

    @Override
    public String getName() {
        return "SimulatedSwerve";
    }
}
