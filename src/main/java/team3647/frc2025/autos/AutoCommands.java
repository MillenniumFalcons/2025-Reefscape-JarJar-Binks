package team3647.frc2025.autos;

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
import team3647.frc2025.Util.PoseUtils;
import team3647.frc2025.constants.AutoConstants;
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

    public Command followChoreoPath(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        autoConstants.xController,
                        autoConstants.yController,
                        autoConstants.rotController),
                (speeds) -> {
                    swerve.drive(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
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
                                            .orElse(autoConstants.kEmptySample)));
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
            return autoConstants.kEmptyTraj;
        }
    }

    @Override
    public void onAllianceFound(Alliance color) {
        this.color = color;
    }

    public boolean isRed() {
        return color == Alliance.Red;
    }

    AutoConstants autoConstants;
    Alliance color;
    SwerveDrive swerve;

    public List<AutoMode> redAutosList =
            List.of(new AutoMode(Commands.none(), new Pose2d(), "nothingRed"));
    public List<AutoMode> blueAutosList =
            List.of(new AutoMode(Commands.none(), new Pose2d(), "nothingBlue"));

    public AutoCommands(AutoConstants autoConstants, SwerveDrive swerve) {
        this.autoConstants = autoConstants;
        color = Alliance.Red;
        this.swerve = swerve;

        // init paths
        this.s3_to_e1 = getTraj("s3 to e1");
        this.e1_to_src = getTraj("e1 to src");
        this.src_to_f2 = getTraj("src to f2");
        this.f2_to_src = getTraj("f2 to src");
        this.src_to_e2 = getTraj("src to e2");
        this.f1_to_src = getTraj("f1 to src");
        this.src_to_f1 = getTraj("src to f1");
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
