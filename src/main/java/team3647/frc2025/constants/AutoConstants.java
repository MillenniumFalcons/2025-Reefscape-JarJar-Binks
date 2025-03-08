package team3647.frc2025.constants;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import java.util.List;

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

    public static final PIDController teleopXController = new PIDController(0.1, 0, 0);
    public static final PIDController teleopYController = new PIDController(0.05, 0, 0);
    public static final PIDController teleopRotController = new PIDController(0.05, 0, 0);

    public static final PIDController AutoXController = new PIDController(6, 0, 0.1);
}
