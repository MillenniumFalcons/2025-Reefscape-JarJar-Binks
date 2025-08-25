package team3647.frc2025.subsystems.wrist;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.WristConstants;

public class SimWrist implements Wrist {
    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);

    // Simulation class to deal w the phyiscs for us, acts like the simulated motor,
    // except we can only give it voltage input
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

    // using wpilib pid controllers to take the place of the talonfx pid controllers
    private final ProfiledPIDController m_controller =
            new ProfiledPIDController(50, 0, 0.5, new TrapezoidProfile.Constraints(5, 10));

    ArmFeedforward m_feedforward =
            new ArmFeedforward(
                    0,
                    0.06,
                    // kv for rads, (idk copeid from 8033)
                    gearbox.KvRadPerSecPerVolt / WristConstants.kGearRatio);

    private final double minAngleRads, maxAngleRads, nominalVoltage, kDt;

    @AutoLog
    public static class PeriodicIO {
        public Pose3d[] armPose = ElevatorConstants.kZeroedElevPose;

        public double position = WristConstants.kStartingAngle.in(Radian);
        public double velocity = 0;
        public double masterCurrent = 0;
        public double demand = position;

        public ControlModeValue controlMode = ControlModeValue.PositionVoltage;
    }

    PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();

    public SimWrist(double minAngleRads, double maxAngleRads, double nominalVoltage, double kDt) {

        if (!Utils.isSimulation())
            throw new IllegalStateException("Trying to run sim wrist on real RIO!!!");
        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;

        AutoLogOutputManager.addObject(this);
    }

    public void setOpenLoop(double percent) {
        periodicIO.demand = percent;
        periodicIO.controlMode = ControlModeValue.DutyCycleOut;
    }

    @Override
    public void setAngle(Angle angle) {
        periodicIO.demand = angle.in(Radian);
        periodicIO.controlMode = ControlModeValue.MotionMagicVoltage;
    }

    @Override
    public void setAngleRads(double angleRads) {
        periodicIO.demand = angleRads;
        periodicIO.controlMode = ControlModeValue.MotionMagicVoltage;
    }

    @Override
    public void setEncoderAngle(double angleRads) {
        Sim.setState(angleRads, 0);
    }

    @Override
    public void setEncoderAngle(Angle angle) {
        setEncoderAngle(angle.in(Radian));
    }

    @Override
    public void writePeriodicOutputs() {

        if (!DriverStationSim.getEnabled()) setOpenLoop(0);

        switch (periodicIO.controlMode) {
            case VoltageOut -> Sim.setInputVoltage(periodicIO.demand);
            case DutyCycleOut -> Sim.setInputVoltage(periodicIO.demand / nominalVoltage);

            case MotionMagicVoltage -> Sim.setInputVoltage(
                    m_controller.calculate(periodicIO.position, periodicIO.demand)
                            + m_feedforward.calculate(
                                    m_controller.getSetpoint().position,
                                    m_controller.getSetpoint().velocity));

            default -> throw new IllegalArgumentException("Unimplemented Control Mode in Wrist!");
        }

        Sim.update(kDt);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.masterCurrent = Sim.getCurrentDrawAmps();
        periodicIO.position = Sim.getAngleRads();
        periodicIO.velocity = Sim.getVelocityRadPerSec();

        // periodicIO.armPose[0] =
        // new Pose3d(
        // ElevatorConstants.kZeroedElevPose[0].getX(),
        // ElevatorConstants.kZeroedElevPose[0].getY(),
        // elevHeightSupplier.get().in(Meter),
        // new Rotation3d(0, periodicIO.position, 0));

        Logger.processInputs(getName(), periodicIO);
    }

    @Override
    public Angle getAngle() {
        return Radian.of(periodicIO.position);
    }

    @Override
    public double getAngleRads() {
        return periodicIO.position;
    }

    @Override
    public double getAngleDegs() {
        return Units.radiansToDegrees(periodicIO.position);
    }

    @Override
    public Angle getMaxAngle() {
        return Radian.of(maxAngleRads);
    }

    @Override
    public Angle getMinAngle() {
        return Radian.of(minAngleRads);
    }

    @Override
    public boolean angleReached(Angle angle, Angle tolerance) {
        return getAngle().minus(angle).abs(Radian) < tolerance.in(Radian);
    }

    @Override
    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleRads() > lowBound && getAngleRads() < highBound;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
