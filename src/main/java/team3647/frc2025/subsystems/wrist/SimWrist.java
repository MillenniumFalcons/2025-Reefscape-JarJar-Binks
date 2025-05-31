package team3647.frc2025.subsystems.wrist;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import team3647.frc2025.Util.InverseKinematics;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.constants.WristConstants;
import team3647.lib.TalonFXSubsystem;

public class SimWrist extends TalonFXSubsystem implements Wrist {

    Angle minAngle, maxAngle;
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);

    private final Supplier<Angle> getPivotAngle;
    private final ProfiledPIDController m_controller =
            new ProfiledPIDController(40, 0, 0.3, new TrapezoidProfile.Constraints(5, 10));
    ArmFeedforward m_feedforward =
            new ArmFeedforward(
                    0,
                    0.27,
                    // kv for rads, (idk copeid from 8033)
                    gearbox.KvRadPerSecPerVolt / WristConstants.kGearRatio);

    private final SingleJointedArmSim Sim =
            new SingleJointedArmSim(
                    gearbox,
                    WristConstants.kGearRatio,
                    WristConstants.kArmMoiKg2M,
                    WristConstants.kArmLengthM,
                    WristConstants.kMinAngle.in(Radian),
                    WristConstants.kMaxAngle.in(Radian),
                    true,
                    WristConstants.kStartingAngle.in(Radian));

    private PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();
    private final double nominalVoltage, kDt;

    public SimWrist(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            Angle minAngle,
            Angle maxAngle,
            Supplier<Distance> elevHeight,
            Supplier<Angle> pivotAngle,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;

        this.getPivotAngle = pivotAngle;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;

        AutoLogOutputManager.addObject(this);
    }

    public void setAngle(Angle angle) {

        periodicIO.demand = angle.in(Radian);
        periodicIO.controlMode = ControlModeValue.MotionMagicVoltage;
    }

    public void setEncoderAngle(Angle angle) {

        setEncoder(angle.in(Degree));
    }

    public Angle getAngle() {
        return Radian.of(periodicIO.position);
    }

    public double getAngleDegs() {
        return periodicIO.position;
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleDegs() > lowBound && getAngleDegs() < highBound;
    }

    public boolean angleReached(double angle, double tolerance) {
        return Math.abs(getAngleDegs() - angle) < tolerance;
    }

    @Override
    public void readPeriodicInputs() {

        switch (periodicIO.controlMode) {
            case VoltageOut -> Sim.setInputVoltage(periodicIO.demand);
            case DutyCycleOut -> Sim.setInputVoltage(periodicIO.demand / nominalVoltage);

            case MotionMagicVoltage -> { Sim.setInputVoltage(
                    m_controller.calculate(periodicIO.position, periodicIO.demand)
                            + m_feedforward.calculate(
                                    m_controller.getSetpoint().position,
                                    m_controller.getSetpoint().velocity));
                }

            default -> throw new IllegalArgumentException("Unimplemented Control Mode in Wrist!");
        }

        Sim.update(kDt);

    }

    public void writePeriodicOutputs(){
        periodicIO.masterCurrent = Sim.getCurrentDrawAmps();
        periodicIO.position = Sim.getAngleRads();
        periodicIO.velocity = Sim.getVelocityRadPerSec();

        Logger.recordOutput("Sim Wrist/Current Angle", getAngleDegs());
        Logger.recordOutput("Sim Wrist/Demand", periodicIO.demand);
        Logger.processInputs(getName(), periodicIO);
    }

    @Override
    public String getName() {
        return "Simulated Wrist";
    }

    @AutoLog
    public static class PeriodicIO {
        public Pose3d wristPose = Pose3d.kZero;

        public double position = WristConstants.kStartingAngle.in(Radian);
        public double velocity = 0;
        public double masterCurrent = 0;
        public double demand = position;

        public ControlModeValue controlMode = ControlModeValue.MotionMagicVoltage;
    }

    @Override
    public Angle getMaxAngle() {

        return WristConstants.kMaxAngle;
    }

}
