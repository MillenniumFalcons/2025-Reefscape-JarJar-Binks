package team3647.frc2025.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import org.dyn4j.collision.narrowphase.FallbackCondition;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import team3647.frc2025.Util.sim.MapleSimSwerveDrivetrain;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.robot.Robot;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveFOCRequest;
import team3647.lib.team254.geometry.Translation2d;
import team3647.lib.team254.swerve.SwerveKinematicLimits;
import team3647.lib.team254.swerve.SwerveSetpoint;
import team3647.lib.team254.swerve.SwerveSetpointGenerator;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.VisionMeasurement;

public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements AllianceObserver,PeriodicSubsystem {

    public final SwerveSetpointGenerator setpointGenerator;

    public final SwerveKinematicLimits limits;


    public final Field2d field = new Field2d();

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final Pose2d zeroPose2d = new Pose2d();

    private final double kDt;

    private double pitchZero = 0;

    private double cachedSpeed = 0;

    private SysIdRoutine m_driveSysIdRoutine;

    private SysIdRoutine m_steerSysIdRoutine;

	private SysIdRoutine spinSysIdRoutine;

	
	Notifier m_simNotifier;

	private static final double kSimLoopPeriod = 0.002;

    // private final DriveTrainSimulationConfig simConfig;
// 
    // private final SelfControlledSwerveDriveSimulation simpleSim;

    // public final SwerveDriveSimulation swerveSim;

	MapleSimSwerveDrivetrain simDrive;

    public static class PeriodicIO {
        // inputs

        public SwerveSetpoint setpoint =
                new SwerveSetpoint(
                        new team3647.lib.team254.swerve.ChassisSpeeds(),
                        new team3647.lib.team254.swerve.SwerveModuleState
                                [4]);

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

        public SwerveModuleState frontLeftState = new SwerveModuleState();
        public SwerveModuleState frontRightState = new SwerveModuleState();
        public SwerveModuleState backLeftState = new SwerveModuleState();
        public SwerveModuleState backRightState = new SwerveModuleState();


        public double timestamp = 0;

        public Pose2d visionPose = new Pose2d();

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds speeds = new ChassisSpeeds();

        public SwerveRequest masterRequest = new SwerveRequest.Idle();
		public SysIdSwerveRotation spinSysidRequest = new SysIdSwerveRotation();
        public SwerveFOCRequest driveFOCRequest = new SwerveFOCRequest(true);
        public SwerveFOCRequest steerFOCRequest = new SwerveFOCRequest(false);
        public FieldCentric fieldCentric =
                new FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        public RobotCentric robotCentric =
                new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public SwerveDrive(
            SwerveDrivetrainConstants swerveDriveConstants,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt,
            // DriveTrainSimulationConfig simConfig,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... swerveModuleConstants
			) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, swerveDriveConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(swerveModuleConstants));
        registerTelemetry(this::setStuff);
        // this.simConfig = simConfig;
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;
		
        // this.swerveSim = new SwerveDriveSimulation(simConfig, new Pose2d(2,2,new Rotation2d()));

        this.setpointGenerator = new SwerveSetpointGenerator(SwerveDriveConstants.kDriveKinematics);

        this.limits = SwerveDriveConstants.kTeleopKinematicLimits;

        // this.simpleSim = new SelfControlledSwerveDriveSimulation(swerveSim);

        this.m_driveSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(10).per(Units.Second),
                                Units.Volts.of(30),
                                Units.Seconds.of(4),
                                ModifiedSignalLogger.logState()),
                        new SysIdRoutine.Mechanism(
                                (Voltage volts) ->
                                        setControl(
                                                periodicIO.driveFOCRequest.withVoltage(
                                                        volts.in(Units.Volts))),
                                null,
                                this));

        this.m_steerSysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Units.Volts.of(3).per(Units.Second),
                                Units.Volts.of(10),
                                Units.Seconds.of(4),
                                ModifiedSignalLogger.logState()),
                        new SysIdRoutine.Mechanism(
                                (Voltage volts) ->
                                        setControl(
                                                periodicIO.steerFOCRequest.withVoltage(
                                                        volts.in(Units.Volts))),
                                null,
                                this));

		this.spinSysIdRoutine = 
				new SysIdRoutine(
					new SysIdRoutine.Config(
						Units.Volts.of(1).per(Second), 
						Units.Volts.of(3), Second.of(10), 
						ModifiedSignalLogger.logState()),
					new Mechanism(
						(Voltage volts) -> setControl(periodicIO.spinSysidRequest.withRotationalRate(volts.in(Units.Volts))), 
						null,
						this));

        // AutoBuilder.configure(
        //     this::getOdoPose, // Robot pose supplier
        //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     (ChassisSpeeds speeds) ->
        //     this.drive(
        //             speeds.vxMetersPerSecond,
        //             speeds.vyMetersPerSecond,
        //             speeds.omegaRadiansPerSecond), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //     new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //             new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //     ),
        //     RobotConfig.fromGUISettings(),
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
        // );

		if(Utils.isSimulation()){
			startSimThread();
		}


        // SimulatedArena.getInstance().addDriveTrainSimulation(simpleSim.getDriveTrainSimulation());
        


            


    }

	private void startSimThread() {
    simDrive = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            // TODO: modify the following constants according to your robot
            Pound.of(125), // robot weight
            Inches.of(30), // bumper length
            Inches.of(30), // bumper width
            DCMotor.getKrakenX60(1), // drive motor type
            DCMotor.getFalcon500(1), // steer motor type
            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(simDrive::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
}

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
		
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = this.getPigeon2().getRoll().getValueAsDouble();
        periodicIO.heading = this.getPigeon2().getYaw().getValueAsDouble();
        periodicIO.pitch = this.getPigeon2().getPitch().getValueAsDouble() - this.pitchZero;
        periodicIO.rawHeading = this.getPigeon2().getYaw().getValueAsDouble();
        periodicIO.frontLeftState = this.getModules()[0].getCurrentState();
        periodicIO.frontRightState = this.getModules()[1].getCurrentState();
        periodicIO.backLeftState = this.getModules()[2].getCurrentState();
        periodicIO.backRightState = this.getModules()[3].getCurrentState();
        periodicIO.gyroRotation = Rotation2d.fromDegrees(periodicIO.heading);
        periodicIO.timestamp = Timer.getFPGATimestamp();

        // SmartDashboard.putBoolean("good", periodicIO.good);

        // SmartDashboard.putNumber("characterization voltage", periodicIO.characterizationVoltage);
    }

    @Override
    public void writePeriodicOutputs() {
        setisAccel();
        this.setControl(periodicIO.masterRequest);
        // simpleSim.periodic();
        // Logger.recordOutput("simRobot/drive", simpleSim.getActualPoseInSimulationWorld());
        Logger.recordOutput("simconsole", "Periodics");
        // SmartDashboard.putNumber("heading", getRawHeading());
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("is accel", getIsAccel());
        // Logger.recordOutput("Robot/Output", this.getOdoPose());
        // Logger.recordOutput(
                // "Drive/Encoder", this.getModules()[0].getDriveMotor().getPosition().getValueAsDouble());
        // Logger.recordOutput(
                // "Drive/Current",
                // this.getModules()[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
    
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void setisAccel() {
        if (getVel() > periodicIO.cachedVel + 0.02) {
            periodicIO.isAccel = true;
        } else {
            periodicIO.isAccel = false;
        }
        periodicIO.cachedVel = getVel();
    }

    public boolean getIsAccel() {
        return periodicIO.isAccel;
    }

    public Command setAccelLimit(double limit) {
        return Commands.runOnce(() -> limits.kMaxDriveAcceleration = limit);
    }

    public void reset() {
        for (int i = 0; i < 4; ++i) {
            periodicIO.setpoint.mModuleStates[i] =
                    new team3647.lib.team254.swerve.SwerveModuleState(
                            0.0,
                            team3647.lib.team254.geometry.Rotation2d.fromRadians(
                                    this.getModulePositions()[i].angle.getRadians()));
        }
        periodicIO.good = true;
    }


    public Command runDriveQuasiTest(Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }

    public void setRobotPose(Pose2d pose) {
		if(RobotBase.isSimulation()){
			simDrive.mapleSimDrive.setSimulationWorldPose(pose);
			Timer.delay(0.05);
		}

        resetPose(pose);
        periodicIO = new PeriodicIO();
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            this.getModules()[i].getEncoder().setPosition(0);
        }
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public void zeroGyro() {
        this.getPigeon2().setYaw(0.0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    @Override
    public void simulationPeriodic() {
        this.updateSimState(kDt, RobotController.getBatteryVoltage());
    }

    private void reduceCancoderStatusframes() {
		
        // this.backLeft.getCanCoderObject().setStatusFramePeriod(CANCoderStatusFrame.SensorData,
        // 255);
        // this.backRight
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        // this.frontLeft
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
        // this.frontRight
        // .getCanCoderObject()
        // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    }

    // Probably want to moving average filter pitch and roll.
    public boolean isBalanced(double thresholdDeg) {
        return Math.abs(getRoll()) < thresholdDeg && Math.abs(getPitch()) < thresholdDeg;
    }

    public double getRawHeading() {
        
        return periodicIO.rawHeading;
    }

    @AutoLogOutput
    public Pose2d getOdoPose() {
        
        // return RobotBase.isReal()? periodicIO.pose : simpleSim.getOdometryEstimatedPose();
		return periodicIO.pose;
        
    }

    public void setStuff(SwerveDriveState state) {
        // SignalLogger.writeDoubleArray(
        //         "odometry",
        //         new double[] {
        //             state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()
        //         });
        periodicIO.pose = state.Pose;
        periodicIO.speeds = this.getKinematics().toChassisSpeeds(state.ModuleStates);
        // SignalLogger.writeDoubleArray(
        //         "speeds",
        //         new double[] {
        //             periodicIO.speeds.vxMetersPerSecond,
        //             periodicIO.speeds.vyMetersPerSecond,
        //             periodicIO.speeds.omegaRadiansPerSecond
        //         });
    }

    public Rotation2d getOdoRot() {
        return getOdoPose().getRotation();
    }

    public double getPoseX() {
        return this.getOdoPose().getX();
    }

    public double getPoseY() {
        return this.getOdoPose().getY();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.getModules()[0].getPosition(true),
            this.getModules()[1].getPosition(true),
            this.getModules()[2].getPosition(true),
            this.getModules()[3].getPosition(true)
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return periodicIO.speeds;
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public boolean shouldAddData(VisionMeasurement measurement) {
        double distance = measurement.pose.minus(getOdoPose()).getTranslation().getNorm();
        double angle =
                measurement.pose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        return (distance < MathUtil.clamp(getVel(), 0.25, 1.5) && Math.abs(angle) < 15)
                        || DriverStation.isAutonomous();
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
        // SmartDashboard.putNumber("timestamped viison", data.timestamp);
        // SignalLogger.writeDoubleArray(
        //         "vision pose",
        //         new double[] {
        //             data.pose.getX(),
        //             data.pose.getY(),
        //             data.pose.getRotation().getDegrees(),
        //             data.timestamp
        //         });
        addVisionMeasurement(data.pose, data.timestamp, data.stdDevs);
        // if(RobotBase.isSimulation()){
        //     simpleSim.addVisionEstimation(data.pose, data.timestamp, data.stdDevs);
            
        // }
    }

    @Override
    public void onAllianceFound(Alliance color) {
        var rot = color == Alliance.Red? Rotation2d.fromRadians(2*Math.PI) : Rotation2d.fromRadians(0);
        setOperatorPerspectiveForward(rot);
    }

    @Override
    public void end() {
        stopModules();
    }

    public void stopModules() {
        periodicIO.masterRequest = new SwerveRequest.Idle();
    }

    public void drive(double x, double y, double rotation) {
        
        
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generateRobotOriented(x, y, rotation);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public SwerveSetpoint generate(double x, double y, double omega) {
        var robotRel =
                team3647.lib.team254.swerve.ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        omega,
                        team3647.lib.team254.geometry.Rotation2d.fromDegrees(
                                this.getOdoPose().getRotation().getDegrees()));
        periodicIO.setpoint =
                setpointGenerator.generateSetpoint(limits, periodicIO.setpoint, robotRel, kDt);
        return periodicIO.setpoint;
    }

    public SwerveSetpoint generateRobotOriented(double x, double y, double omega) {
        var robotRel = new team3647.lib.team254.swerve.ChassisSpeeds(x, y, omega);
        periodicIO.setpoint =
                setpointGenerator.generateSetpoint(limits, periodicIO.setpoint, robotRel, kDt);
        return periodicIO.setpoint;
    }

    public void driveFieldOriented(double x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generate(x, y, rotation);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
        // if(RobotBase.isSimulation()){
            
        //     simpleSim.runChassisSpeeds(
        //         setpoint.mChassisSpeeds.real(), 
        //         new Translation2d().real(), true, true);
                
        // }
        
    }

    // public void resetSimOdo(){
    //     simpleSim.resetOdometry(simpleSim.getActualPoseInSimulationWorld());
    // }

    public void driveFieldOriented(DoubleSupplier x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generate(x.getAsDouble(), y, rotation);
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public void driveFieldOriented(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint =
                generate(x.getAsDouble(), y.getAsDouble(), rotation.getAsDouble());
        periodicIO
                .robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        periodicIO.masterRequest = periodicIO.robotCentric;
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            periodicIO.frontLeftState,
            periodicIO.frontRightState,
            periodicIO.backLeftState,
            periodicIO.backRightState
        };
    }

    public double getVel() { // squared
        return this.getChassisSpeeds().vxMetersPerSecond * this.getChassisSpeeds().vxMetersPerSecond
                + this.getChassisSpeeds().vyMetersPerSecond
                        * this.getChassisSpeeds().vyMetersPerSecond;
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }

    @Override
    public String getName() {
        return "Swerve Drivetrain";
    }
}
