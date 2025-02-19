package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
    // motor rot * 243/12160 = intake rot
    // relative to reset position all the way up = up = 0
    // motor native rotations:
    // up: 0
    // down: -29.491211
    // 90 deg: -3.726562
    // 0 deg: -27.275879
    // intake rot:
    // up: 0
    // down: -0.58933916718
    // 90 deg: -0.07446994786
    // 0 deg: -0.54506896356
    // 0.4705990157/90 deg = intake rot per degree = 0.00522887795
    // 0.26165907777 = native rot per degree
    // 14.991960828 = native rot per radian
    public static final Angle kMaxAngle = Radians.of(1.6715990135);
    public static final Angle kMinAngle = Radians.of(-0.14776799548); // this is the ground
    public static final Angle kStartingAngle = kMaxAngle;

    public static final double kNativeToRad = 1 / 14.991960828;

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.WristIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMaster.setPosition(0);

        // needs tuning when the motor finally works
        kMasterConfig.Slot0.withKP(0);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        kMasterConfig.Slot0.withKS(0);
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0);

        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
