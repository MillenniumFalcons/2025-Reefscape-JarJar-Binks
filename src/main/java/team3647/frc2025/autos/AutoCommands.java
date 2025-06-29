package team3647.frc2025.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3647.frc2025.Util.AllianceFlip;
import team3647.frc2025.Util.PoseUtils;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.Util.AutoDrive.DriveMode;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.subsystems.Drivetrain.SwerveDrive;
import team3647.frc2025.subsystems.superstructure.Superstructure;
import team3647.lib.team9442.AllianceObserver;

public class AutoCommands implements AllianceObserver {
    private final Trajectory<SwerveSample> s3_to_e1;
    private final Trajectory<SwerveSample> e1_to_src;
    private final Trajectory<SwerveSample> src_to_f1;
    private final Trajectory<SwerveSample> f1_to_src;
    private final Trajectory<SwerveSample> src_to_f2;
    private final Trajectory<SwerveSample> f2_to_src;
    private final Trajectory<SwerveSample> src_to_e2;
    private final Trajectory<SwerveSample> s2_to_d2;
    private final Trajectory<SwerveSample> s4_to_f2;
    private final Trajectory<SwerveSample> f2_to_lollipop3;
    private final Trajectory<SwerveSample> lollipop3_to_a1;
    private final Trajectory<SwerveSample>  a1_to_lollipop2;
    private final Trajectory<SwerveSample>  lollipop2_to_a2;
    private final Trajectory<SwerveSample> a2_to_lollipop1;
    private final Trajectory<SwerveSample> lollipop1_to_b1;
    
        private final Rectangle2d BlueLSource =
                new Rectangle2d(
                        new Translation2d(0, FieldConstants.kFieldLengthM),
                        new Translation2d(2.5893, 5.4657));
        private final Rectangle2d BlueRSource =
                new Rectangle2d(Translation2d.kZero, new Translation2d(2.5893, 1.192));
        private final Rectangle2d RedLSource =
                new Rectangle2d(
                        AllianceFlip.flip(new Translation2d(0, FieldConstants.kFieldLengthM)),
                        AllianceFlip.flip(new Translation2d(2.5893, 5.4657)));
        private final Rectangle2d RedRSource =
                new Rectangle2d(
                        AllianceFlip.flip(Translation2d.kZero),
                        AllianceFlip.flip(new Translation2d(2.5893, 1.192)));
    
        private final PIDController rotController = new PIDController(5, 0, 0);
    
        public Command getFour_s3e1f1f2e2() {
            return Commands.sequence(
                    followChoreoPath(s3_to_e1),
                    Commands.waitSeconds(1),
                    followChoreoPath(e1_to_src),
                    Commands.waitSeconds(1),
                    followChoreoPath(src_to_f2),
                    Commands.waitSeconds(1),
                    followChoreoPath(f2_to_src),
                    Commands.waitSeconds(1),
                    followChoreoPath(src_to_f1),
                    Commands.waitSeconds(1),
                    followChoreoPath(f1_to_src),
                    Commands.waitSeconds(1),
                    followChoreoPath(src_to_e2));
        }
    
        public Command getSuperstructureOneS2_d2() {
            return Commands.sequence(
                Commands.parallel(
                                superstructure.goToState(SuperstructureState.preloadScore),
                                Commands.run(()->
                                                superstructure
                                                        .coralerCommands
                                                        .setOpenLoop(0.1)
                                                        .withTimeout(1)))
                        .withTimeout(10),
                Commands.waitSeconds(3),
                superstructure.Stow().alongWith(superstructure.poopCoral()));
        }
    
        public Command getSuperstructureTwoS3_e2f1() {
            return Commands.sequence(
                    scoreCoral(),
                    Commands.repeatingSequence(
                            superstructure.intake().until(superstructure::intakeCurrent)));
        }
    
        // public Command masterSuperstructureSequence(){
        // 	return Commands.sequence(
        // 		scoreCoral(),
        // 		Commands.repeatingSequence(
        // 			superstructure.intake().until(intakeCurrent),
        // 			superstructure.transfer().until(seagullCurrent),
        // 			superstructure.handoff().until(coralerCurrent),
    
        // 		)
        // 	);
        // }
    
        // public Command scorePreload() {
        //     return Commands.sequence(
        //             Commands.parallel(
        //                             superstructure.goToState(() -> SuperstructureState.preloadScore),
        //                             Commands.run( ()->
        //                                             superstructure
        //                                                     .coralerCommands
        //                                                     .setOpenLoop(0.1)
        //                                                     .withTimeout(1)))
        //                     .withTimeout(10),
        //             Commands.waitSeconds(3),
        //             superstructure.Stow().alongWith(superstructure.poopCoral()));
        // }

        public Command scoreCoral() {
                return Commands.deadline(
                        superstructure.poopCoral().withTimeout(0.2),
                        Commands.waitSeconds(0.05).andThen(superstructure.goToState(SuperstructureState.Stow, true)),
                        Commands.waitSeconds(0.25).andThen(
                                Commands.run(()-> 
                                        swerve.drive(-0.4, 0, 0),
                                        swerve
                                )
                        )
                );
                        // superstructure.poopCoral().andThen(
                        // Commands.parallel(
                        // superstructure.goToState(SuperstructureState.Stow, true),
                        // Commands.run(()-> swerve.drive(-0.4, 0, 0), swerve).withTimeout(0.25)));
        }
    
        public Command autoAlign(ScoringPos wantedPos){
                return Commands.run(() -> {
                        Twist2d vels = getAutoDriveVels.apply(DriveMode.SCORE, wantedPos);
                        swerve.drive(
                                vels.dx,
                                vels.dy,
                                vels.dtheta
                        );
                }, swerve).until(() -> PoseUtils.inCircle(swerve.getOdoPose(), wantedPos.pose, 0.01));
        }
        public Command getOneS2_d2() {
            return Commands.parallel(
                    Commands.runOnce(() -> {
                            superstructure.preloaded = true;
                            superstructure.setPeice();
                        }),
                     superstructure.goToState(SuperstructureState.preloadScore), followChoreoPathSlow(s2_to_d2))
                     .andThen(scoreCoral())
                     .andThen(Commands.runOnce(() -> superstructure.preloaded = false));
        }
    
        public Command getOneDT_s2e2() {
            return followChoreoPath(s2_to_d2);
        }

        public Command getFourS4_f2_a1_a2_b1(){
                return Commands.parallel(
                        Commands.runOnce(() -> {
                                superstructure.preloaded = true;
                                superstructure.setPeice();
                            }),
                        superstructure.goToState(SuperstructureState.preloadScore), followChoreoPath(s4_to_f2))
                        .andThen(autoAlign(ScoringPos.PreloadF2BRUH),
                        scoreCoral().alongWith(Commands.runOnce(()-> superstructure.preloaded = false)))
                        .andThen(
                                Commands.parallel(
                                        superstructure.intake(true).until(superstructure::hasPeice).andThen(superstructure.goToState(SuperstructureState.HighScore)),
                                        followChoreoPath(f2_to_lollipop3).until(superstructure::hasPeice).andThen(
                                        followChoreoPath(lollipop3_to_a1)))) 
                        .andThen(
                                autoAlign(ScoringPos.A1),
                                scoreCoral())
                        .andThen(
                                Commands.parallel(
                                        superstructure.intake(true).until(superstructure::hasPeice).andThen(superstructure.goToState(SuperstructureState.HighScore)),
                                        followChoreoPath(a1_to_lollipop2).until(superstructure::hasPeice).andThen(
                                        followChoreoPath(lollipop2_to_a2))))
                        .andThen(
                                autoAlign(ScoringPos.A2),
                                scoreCoral()
                        ).andThen(
                                Commands.parallel(
                                        superstructure.intake(true).until(superstructure::hasPeice).andThen(superstructure.goToState(SuperstructureState.HighScore)),
                                        followChoreoPath(a2_to_lollipop1).until(superstructure::hasPeice).andThen(
                                        followChoreoPath(lollipop1_to_b1)))
                        ).andThen(
                                autoAlign(ScoringPos.B1),
                                Commands.parallel(
                                        superstructure.poopCoral().withTimeout(0.2),
                                        Commands.waitSeconds(0.05).andThen(superstructure.goToState(SuperstructureState.Stow, true)),
                                        Commands.waitSeconds(0.25).andThen(
                                                Commands.run(()-> 
                                                        swerve.drive(-0.4, 0, 0),
                                                        swerve
                                                ).withTimeout(0.25)
                        )
                        ));
        }
    
        public Command followChoreoPath(Trajectory<SwerveSample> traj) {
            return customPathFollower(
                    traj,
                    choreoSwerveController(
                            AutoConstants.autoXController,
                            AutoConstants.autoYController,
                            rotController),
                    (speeds) -> {
                        swerve.drive(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond,
                                speeds.omegaRadiansPerSecond);
                    },
                    swerve::getOdoPose);
        }
    
        public Command followChoreoPathWithOverrideFast(Trajectory<SwerveSample> traj) {
            try {
                PathPlannerLogging.logActivePath(PathPlannerPath.fromChoreoTrajectory(traj.name()));
            } catch (Exception e) {
            }
    
            // Logger.recordOutput("Autos/current path", path);
            return customPathFollower(
                            traj,
                            choreoSwerveController(
                                    AutoConstants.autoXController,
                                    AutoConstants.autoYController,
                                    AutoConstants.rotController),
                            (ChassisSpeeds speeds) -> {
                                var motionRotComponent = speeds.omegaRadiansPerSecond;
                                var motionXComponent = speeds.vxMetersPerSecond;
                                var motionYComponent = speeds.vyMetersPerSecond;
    
                                var isInPose =
                                        PoseUtils.inRect(swerve.getOdoPose(), BlueLSource)
                                                || PoseUtils.inRect(swerve.getOdoPose(), BlueRSource)
                                                || PoseUtils.inRect(swerve.getOdoPose(), RedLSource)
                                                || PoseUtils.inRect(swerve.getOdoPose(), RedRSource);
    
                                if (!this.intakeCurrent.getAsBoolean()
                                        && hasTarget.getAsBoolean()
                                        && isInPose) {
                                    motionXComponent = getAutoDriveVels.apply(DriveMode.INTAKE, ScoringPos.NONE).dx;
                                    motionYComponent = getAutoDriveVels.apply(DriveMode.INTAKE, ScoringPos.NONE).dy;
                                    // motionRotComponent = autoDriveVelocities.get().dtheta;
                                }
    
                                swerve.drive(motionXComponent, motionYComponent, motionRotComponent);
                            },
                            swerve::getOdoPose)
                    .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
        }
    
        public Command followChoreoPathSlow(Trajectory<SwerveSample> traj) {
            return customPathFollower(
                    traj,
                    choreoSwerveController(
                            AutoConstants.autoXController,
                            AutoConstants.autoYController,
                            rotController),
                    (speeds) -> {
                        swerve.drive(
                                speeds.vxMetersPerSecond * 0.5,
                                speeds.vyMetersPerSecond * 0.5,
                                speeds.omegaRadiansPerSecond * 0.5);
                    },
                    swerve::getOdoPose);
        }
    
        public Command customPathFollower(
                Trajectory<SwerveSample> traj,
                ChoreoController controller,
                Consumer<ChassisSpeeds> outputChassisSpeeds,
                Supplier<Pose2d> poseSupplier) {
    
            Timer timer = new Timer();
    
            return new FunctionalCommand(
                    timer::restart,
                    () -> {
                        outputChassisSpeeds.accept(
                                controller.apply(
                                        poseSupplier.get(),
                                        traj.sampleAt(timer.get(), isRed())
                                                .orElse(AutoConstants.kEmptySample)));
                    },
                    (interrupted) -> {
                        outputChassisSpeeds.accept(new ChassisSpeeds());
                        timer.stop();
                    },
                    () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.05),
                    swerve);
        }
        
    
    
        public static ChoreoController choreoSwerveController(
                PIDController xController,
                PIDController yController,
                PIDController rotationController) {
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            return (pose, referenceState) -> {
                double xFF = referenceState.getChassisSpeeds().vxMetersPerSecond;
                double yFF = referenceState.getChassisSpeeds().vyMetersPerSecond;
                double rotationFF = referenceState.getChassisSpeeds().omegaRadiansPerSecond;
    
                double xFeedback = xController.calculate(pose.getX(), referenceState.getPose().getX());
                double yFeedback = yController.calculate(pose.getY(), referenceState.getPose().getY());
                double rotationFeedback =
                        rotationController.calculate(
                                pose.getRotation().getRadians(),
                                referenceState.getPose().getRotation().getRadians());
    
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                        xFF + xFeedback,
                        yFF + yFeedback,
                        rotationFF + rotationFeedback,
                        pose.getRotation());
            };
        }

        public static ChoreoController choreoSwerveController(
                PIDController xController,
                PIDController yController,
                ProfiledPIDController rotationController) {
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            return (pose, referenceState) -> {
                double xFF = referenceState.getChassisSpeeds().vxMetersPerSecond;
                double yFF = referenceState.getChassisSpeeds().vyMetersPerSecond;
                double rotationFF = referenceState.getChassisSpeeds().omegaRadiansPerSecond;
    
                double xFeedback = xController.calculate(pose.getX(), referenceState.getPose().getX());
                double yFeedback = yController.calculate(pose.getY(), referenceState.getPose().getY());
                double rotationFeedback =
                        rotationController.calculate(
                                pose.getRotation().getRadians(),
                                referenceState.getPose().getRotation().getRadians());
    
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                        xFF + xFeedback,
                        yFF + yFeedback,
                        rotationFF + rotationFeedback,
                        pose.getRotation());
            };
        }
    
        public Trajectory<SwerveSample> getTraj(String path) {
            var trajMaybe = Choreo.loadTrajectory(path);
    
            if (trajMaybe.isPresent()) {
                return (Trajectory<SwerveSample>) trajMaybe.get();
            } else {
                return AutoConstants.kEmptyTraj;
            }
        }
    
        public Pose2d getInitial(Trajectory<SwerveSample> traj, Alliance color) {
    
            return traj.getInitialPose(color == Alliance.Red).orElse(new Pose2d());
        }
    
        @Override
        public void onAllianceFound(Alliance color) {

                Logger.recordOutput("Alliance Change!", Timer.getFPGATimestamp());
    
            this.color = color;
                setIsRed();
    }

    public boolean isRed() {

        return this.isRed;
    }

    public void setIsRed() {
        this.isRed = color == Alliance.Red;
    }

    Alliance color;
    boolean isRed = true;
    SwerveDrive swerve;
    Superstructure superstructure;
    BiFunction<DriveMode, ScoringPos, Twist2d> getAutoDriveVels;

    Trigger intakeCurrent;
    Trigger coralerCurrent;
    Trigger seagullCurrent;

    private final BooleanSupplier hasTarget;

    public final AutoMode blueOne_s2d2;

    public final AutoMode blueFour_s4f2a1a2b1;

    public final AutoMode blueDT_One_s2d2;

    public final AutoMode redOne_s2d2;

    public final AutoMode redDT_One_s2d2;

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            BiFunction<DriveMode, ScoringPos, Twist2d> getAutoDriveVels,
            BooleanSupplier hasTarget) {
        this.superstructure = superstructure;
        color = Alliance.Red;
        
        this.swerve = swerve;
        this.getAutoDriveVels = getAutoDriveVels;
        this.hasTarget = hasTarget;

        // init paths
        this.s3_to_e1 = getTraj("s3 to e1");
        this.e1_to_src = getTraj("e1 to src");
        this.src_to_f2 = getTraj("src to f2");
        this.f2_to_src = getTraj("f2 to src");
        this.src_to_e2 = getTraj("src to e2");
        this.f1_to_src = getTraj("f1 to src");
        this.src_to_f1 = getTraj("src to f1");
        this.s2_to_d2 = getTraj("s2 to d2");
        this.s4_to_f2 = getTraj("s4 to f2");
        this.f2_to_lollipop3 = getTraj("f2 to lollipop3");
        this.lollipop3_to_a1 = getTraj("lollipop3 to a1");
        this.a1_to_lollipop2 = getTraj("a1 to lollipop2");
        this.lollipop2_to_a2 = getTraj("lollipop2 to a2");
        this.a2_to_lollipop1 = getTraj("a2 to lollipop1");
        this.lollipop1_to_b1 = getTraj("lollipop1 to b1");

        this.blueOne_s2d2 =
                new AutoMode(
                        getOneS2_d2(), getInitial(s2_to_d2, Alliance.Blue), "blue one Piece mid");

        this.blueDT_One_s2d2 =
                new AutoMode(
                        getOneDT_s2e2(),
                        getInitial(s2_to_d2, Alliance.Blue),
                        "blue one piece mid ONLY DT");

        this.blueFour_s4f2a1a2b1 = new AutoMode(
                getFourS4_f2_a1_a2_b1(), getInitial(s4_to_f2, Alliance.Blue), "blue four piece top"
        );

        this.redDT_One_s2d2 =
                new AutoMode(
                        getOneDT_s2e2(),
                        getInitial(s2_to_d2, Alliance.Red),
                        "red one piece mid ONLY DT");
        this.redOne_s2d2 =
                new AutoMode(
                        getOneS2_d2(), getInitial(s2_to_d2, Alliance.Red), "red one Piece mid");

        
        

        blueAutosList =
                List.of(
                        new AutoMode(
                                Commands.none(),
                                Pose2d.kZero.rotateBy(Rotation2d.k180deg),
                                "nothingBlue"),
                        blueOne_s2d2,
                        blueDT_One_s2d2,
                        blueFour_s4f2a1a2b1);
        redAutosList =
                List.of(
                        new AutoMode(Commands.none(), Pose2d.kZero, "nothingRed"),
                        redOne_s2d2,
                        redDT_One_s2d2);

        this.coralerCurrent = new Trigger(superstructure.coralerCommands.current()).debounce(0.4);
        this.intakeCurrent = new Trigger(superstructure::intakeCurrent).debounce(0.5);
        this.seagullCurrent = new Trigger(superstructure::seagullCurrent).debounce(0.5);

    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
