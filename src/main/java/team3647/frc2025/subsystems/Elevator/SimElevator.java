// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import team3647.frc2025.constants.ElevatorConstants;

public class SimElevator implements Elevator {
    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2);

    // Standard classes for controlling our elevator
    private final ProfiledPIDController m_controller =
            new ProfiledPIDController(50, 0, 0, new TrapezoidProfile.Constraints(5, 10));
    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    0,
                    0.06,
                    // kv for meters, (idk copeid from 8033)
                    m_elevatorGearbox.KvRadPerSecPerVolt
                            * ElevatorConstants.kElevatorDrumRadius
                            / ElevatorConstants.kGearRatio);

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim Sim =
            new ElevatorSim(
                    m_elevatorGearbox,
                    ElevatorConstants.kGearRatio,
                    ElevatorConstants.kCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kMinHeight.in(Meters),
                    ElevatorConstants.kMaxHeight.in(Meters),
                    true,
                    ElevatorConstants.kStartingHeight.in(Meters));

    public LoggedMechanism2d Mech2d = new LoggedMechanism2d(Units.inchesToMeters(2), 1.8288);
    public LoggedMechanismRoot2d Mech2dRoot =
            Mech2d.getRoot("Elevator Root", Units.inchesToMeters(1 - 0.7185), 0);
    public LoggedMechanismLigament2d ElevatorMech2d =
            Mech2dRoot.append(
                    new LoggedMechanismLigament2d("Elevator", Sim.getPositionMeters(), 90));

    @AutoLog
    public static class PeriodicIO {
        public Pose3d[] elevatorPose = ElevatorConstants.kZeroedElevPose;

        public double position = ElevatorConstants.kStartingHeight.in(Meters);
        public double velocity = 0;
        public double masterCurrent = 0;
        public double demand = position;

        public ControlModeValue controlMode = ControlModeValue.PositionVoltage;
    }

    private final double kDt;
    private final double minLengthM;
    private final double maxLengthM;
    private final double nominalVoltage;

    PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();

    /** Subsystem constructor. */
    public SimElevator(double minLength, double maxLength, double nominalVoltage, double kDt) {
        if (!Utils.isSimulation())
            throw new IllegalStateException("TRYING TO RUN SIM ELEVATOR ON REAL RIO");

        this.minLengthM = minLength;
        this.maxLengthM = maxLength;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;
        AutoLogOutputManager.addObject(this);
    }

    @Override
    public void setOpenLoop(double output) {
        periodicIO.demand = output;
        periodicIO.controlMode = ControlModeValue.DutyCycleOut;
    }

    public void setVoltage(double volts) {
        periodicIO.demand = volts;
        periodicIO.controlMode = ControlModeValue.VoltageOut;
    }

    @Override
    public void setHeight(double hieght) {
        periodicIO.demand = MathUtil.clamp(hieght, this.minLengthM, this.maxLengthM);
        periodicIO.controlMode = ControlModeValue.MotionMagicVoltage;
    }

    public void setCurrent(double Amps) {
        periodicIO.demand = Amps;
        periodicIO.controlMode = ControlModeValue.TorqueCurrentFOC;
    }

    @Override
    public void setEncoderHeight(double height) {
        Sim.setState(height, 0);
    }

    @Override
    public void setHeightNative(double height) {
        setHeight(height);
    }

    @Override
    public void setHeight(Distance height) {
        setHeight(height.in(Meters));
    }

    public void disable() {
        setOpenLoop(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (!DriverStationSim.getEnabled()) {
            disable();
        }

        switch (periodicIO.controlMode) {
            case DutyCycleOut -> Sim.setInputVoltage(periodicIO.demand / this.nominalVoltage);

            case VoltageOut -> Sim.setInputVoltage(periodicIO.demand);

            case MotionMagicVoltage -> Sim.setInputVoltage(
                    m_controller.calculate(periodicIO.position, periodicIO.demand)
                            + m_feedforward.calculate(m_controller.getSetpoint().velocity));

            case TorqueCurrentFOC -> Sim.setInputVoltage(
                    m_elevatorGearbox.getVoltage(
                            periodicIO.demand,
                            Sim.getVelocityMetersPerSecond()
                                    / ElevatorConstants.kElevatorDrumRadius));

            default -> throw new IllegalArgumentException(
                    "Non Implemented Control Mode " + periodicIO.controlMode);
        }

        Sim.update(kDt);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = this.Sim.getPositionMeters();
        periodicIO.velocity = this.Sim.getVelocityMetersPerSecond();
        periodicIO.masterCurrent = this.Sim.getCurrentDrawAmps();
        // periodicIO.elevatorPose[0] =
        //         new Pose3d(
        //                 ElevatorConstants.kZeroedElevPose[0].getX(),
        //                 ElevatorConstants.kZeroedElevPose[0].getY(),
        //                 periodicIO.position,
        //                 Rotation3d.kZero);

        ElevatorMech2d.setLength(periodicIO.position);
        Logger.recordOutput(getName() + "/ Mechanism2d", Mech2d);
        Logger.processInputs(getName(), periodicIO);
    }

    @Override
    public boolean heightReached(Distance height, Distance tolerance) {
        return getHeight().minus(height).abs(Meters) < tolerance.in(Meters);
    }

    @Override
    public Distance getHeight() {
        return Meters.of(periodicIO.position);
    }

    @Override
    public String getName() {

        return "Elevator";
    }
}
