package team3647.frc2025.constants;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import java.util.List;

public class AutoConstants {

    public static SwerveSample kEmptySample =
            new SwerveSample(-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0}, new double[] {0});
    public static EventMarker kEmptyEventMarker = new EventMarker(-1, "nothing");

    public static Trajectory<SwerveSample> kEmptyTraj =
            new Trajectory<SwerveSample>(
                    "kEmtpy", List.of(kEmptySample), List.of(1), List.of(kEmptyEventMarker));

    public static final ProfiledPIDController xController = new ProfiledPIDController(1.5, 0, 0, new Constraints(5, 10));
    public static final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0.1, new Constraints(5, 10));
    public static final PIDController rotController = new PIDController(5, 0, 0);
    public static final PIDController autoXController = new PIDController(1.5, 0, 0);
    public static final PIDController autoYController = new PIDController(4, 0, 0.1);
    

}
