package team3647.frc2025.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {
    public static final double kDt = 0.02;

    public static final Mass kRobotMass = Units.Kilogram.of(125);
    public static final MomentOfInertia kRobotMoi = Units.KilogramSquareMeters.of(3.20472599);

    public static final String kDriveCanbusName = "drive";
    public static final String kSubsystemCanbusName = "subsystems";

    public static double kNominalVoltage = 11.0;

    // obv need tuinng theyre alll 0
    public class PivotIds {
        public static final int kMasterId = 21;
        public static final int kFollowerId = 55;
    }

    public class ElevatorIds {
		public static final int kSlaveId = 55;
        public static final int kMasterId = 22;
    }

    public class CoralerIds {
        public static final int kMasterId = 20;
    }

    public class IntakeIds {
        public static final int kMasterId = 55;
    }

    public class WristIds {
        public static final int kMasterId = 55;
    }
}
