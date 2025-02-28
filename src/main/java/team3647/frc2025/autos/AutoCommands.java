package team3647.frc2025.autos;

import static edu.wpi.first.units.Units.Degree;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.List;
// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.Util.PoseUtils;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Superstructure;
import team3647.frc2025.subsystems.SwerveDrive;
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
        return scorePreload();
    }

    public Command scorePreload() {
        return Commands.sequence(
                Commands.parallel(superstructure.wristCommands.setAngle(Degree.of(95))),
                Commands.sequence(
                        superstructure.clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                        Commands.parallel(
                                superstructure.elevatorCommands.setHeight(
                                        ElevatorConstants.kLevel4Height),
                                superstructure.pivotCommands.setAngle(
                                        PivotConstants.kStowAngleUp))),
                Commands.waitSeconds(0.5),
                superstructure.scoreL4(),
                superstructure.stowFromL4());
    }

    public Command getSuperstructureTwoS3_e1f2() {
        return Commands.sequence(
                scorePreload().alongWith(autoDrive.setWantedScoringPos(ScoringPos.E1)),
                Commands.waitSeconds(0.1),
                superstructure
                        .wristCommands
                        .setAngle(WristConstants.kSourceIntakeAngle)
                        .alongWith(superstructure.rollersCommands.setOpenLoop(-0.3))
                        .until(superstructure.intakeCurrent()),
                superstructure
                        .stowFromIntake()
                        .alongWith(autoDrive.setWantedScoringPos(ScoringPos.F2)),
                Commands.waitSeconds(1),
                superstructure.prepL4().alongWith(superstructure.wristCommands.stow()),
                superstructure.scoreL4(),
                superstructure.stowFromL4());
    }

    public Command getTwoS3_e1f2() {
        return Commands.parallel(
                getSuperstructureTwoS3_e1f2(),
                Commands.sequence(
                        followChoreoPath(s3_to_e1),
                        Commands.waitSeconds(3),
                        followChoreoPath(e1_to_src),
                        Commands.waitSeconds(3),
                        followChoreoPath(src_to_f2)));
    }

    public Command getOneS2_d2() {
        return Commands.parallel(getSuperstructureOneS2_d2(), followChoreoPathSlow(s2_to_d2));
    }

    public Command getOneDT_s2e2() {
        return followChoreoPath(s2_to_d2);
    }

    public Command followChoreoPath(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        AutoConstants.AutoXController, AutoConstants.yController, rotController),
                (speeds) -> {
                    swerve.drive(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                },
                swerve::getOdoPose);
    }

    public Command followChoreoPathSlow(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        AutoConstants.AutoXController, AutoConstants.yController, rotController),
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
                () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.1),
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
    AutoDrive autoDrive;

    public final AutoMode blueOne_s2d2;
    public final AutoMode blueTwoS3_e1f2;
    public final AutoMode DT_One_s2d2;

    public final AutoMode redOne_s2d2;
    public final AutoMode redTwoS3_e1f2;

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public AutoCommands(SwerveDrive swerve, Superstructure superstructure, AutoDrive autoDrive) {
        this.superstructure = superstructure;
        color = Alliance.Red;

        this.swerve = swerve;
        this.autoDrive = autoDrive;

        // init paths
        this.s3_to_e1 = getTraj("s3 to e1");
        this.e1_to_src = getTraj("e1 to src");
        this.src_to_f2 = getTraj("src to f2");
        this.f2_to_src = getTraj("f2 to src");
        this.src_to_e2 = getTraj("src to e2");
        this.f1_to_src = getTraj("f1 to src");
        this.src_to_f1 = getTraj("src to f1");
        this.s2_to_d2 = getTraj("s2 to d2");

        this.blueOne_s2d2 =
                new AutoMode(
                        getOneS2_d2(), getInitial(s2_to_d2, Alliance.Blue), "blue one Piece mid");
        this.blueTwoS3_e1f2 =
                new AutoMode(
                        getTwoS3_e1f2(),
                        getInitial(s3_to_e1, Alliance.Blue),
                        "blue two piece Left");
        this.DT_One_s2d2 =
                new AutoMode(
                        getOneDT_s2e2(),
                        getInitial(s2_to_d2, Alliance.Blue),
                        "blue one piece mid ONLY DT");

        this.redOne_s2d2 =
                new AutoMode(
                        getOneS2_d2(), getInitial(s2_to_d2, Alliance.Red), "red one Piece mid");
        this.redTwoS3_e1f2 =
                new AutoMode(
                        getTwoS3_e1f2(), getInitial(s3_to_e1, Alliance.Red), "red one Piece Left");

        blueAutosList =
                List.of(
                        new AutoMode(Commands.none(), new Pose2d(), "nothingBlue"),
                        blueOne_s2d2,
                        blueTwoS3_e1f2);
        redAutosList =
                List.of(
                        new AutoMode(Commands.none(), new Pose2d(), "nothingRed"),
                        redOne_s2d2,
                        redTwoS3_e1f2);
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
