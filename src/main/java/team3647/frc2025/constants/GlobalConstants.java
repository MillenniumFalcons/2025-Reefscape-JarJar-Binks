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

    public class PivotIds {
        //unknown bc electrical
        public static final int kMasterId = 21;
    }

    public class ElevatorIds {
        public static final int kSlaveId = 31;
        public static final int kMasterId = 32;
    }

    public class CoralerIds {
        public static final int kMasterId = 26;
    }

    public class WristIds {
        public static final int kMasterId = 30;
    }

    public class RollersIds {
        public static final int kMasterId = 36;
        //public static final int kSeagullId = 40;
    }

    public class ClimbIds {
        //also dont know
        public static final int kMasterId = 26;
    }

    public class KickerIds {
        // ts is a random guess change to real one later
        public static final int kMasterId = 20;
    }
}
