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

    public static final PIDController xController = new PIDController(5, 0, 0.1);
    public static final PIDController yController = new PIDController(4, 0, 0.1);
    public static final PIDController rotController = new PIDController(5, 0, 0);

	public static final PIDController AutoXController = new PIDController(6, 0, 0.1);

}
