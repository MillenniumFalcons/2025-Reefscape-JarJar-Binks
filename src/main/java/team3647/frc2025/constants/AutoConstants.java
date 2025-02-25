package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;

public class AutoConstants {

    public static SwerveSample kEmptySample =
            new SwerveSample(-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0}, new double[] {0});
    public static EventMarker kEmptyEventMarker = new EventMarker(-1, "nothing");

    public static Trajectory<SwerveSample> kEmptyTraj =
            new Trajectory<SwerveSample>(
                    "kEmtpy", List.of(kEmptySample), List.of(1), List.of(kEmptyEventMarker));

    public static PIDController xController = new PIDController(4, 0, 0);
    public static PIDController yController = new PIDController(4, 0, 0);
    public static PIDController rotController = new PIDController(5, 0, 0);

    public static ModuleConfig ppModuleConfig =
            new ModuleConfig(
                    Meters.of(TunerConstants.BackLeft.WheelRadius),
                    TunerConstants.kSpeedAt12Volts,
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                    DCMotor.getKrakenX60Foc(1).withReduction(5.684210526315789),
                    Units.Amps.of(90),
                    1);

    public static RobotConfig ppRobotConfig;

    static {
        try {
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ppRobotConfig =
                    new RobotConfig(
                            Kilogram.of(74),
                            KilogramSquareMeters.of(6.883),
                            ppModuleConfig,
                            new Translation2d[] {
                                new Translation2d(),
                                new Translation2d(),
                                new Translation2d(),
                                new Translation2d()
                            });

            DriverStation.reportError(
                    "problem setting pp robot config from gui", e.getStackTrace());
        }
    }
}
