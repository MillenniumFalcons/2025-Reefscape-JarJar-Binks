package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;

public class Elevator extends TalonFXSubsystem {

    final double minLength, maxLength, kG;

	public final SysIdRoutine sysid, voltageSysid;

	// public final ElevatorSim sim;

	// TalonFXSimState masterSim;

    public Elevator(
            TalonFX master,
            TalonFX follower,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kG,
            double minLength,
            double maxLength,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        super.addFollower(follower, false);
        this.minLength = minLength;
        this.maxLength = maxLength;
        this.kG = kG;

		this.sysid = new SysIdRoutine(
			new Config(
				Units.Volts.of(6).per(Units.Second), 
				Units.Volts.of(30), 
				Units.Seconds.of(10), 
				ModifiedSignalLogger.logState()), 
			new Mechanism(
				(volts) -> setTorque(volts.in(Units.Volt)), 
				null, 
				this,
				"elevator torque sysid"));

		this.voltageSysid = new SysIdRoutine(
			new Config(
				Units.Volts.of(0.9).per(Units.Second),
				Units.Volt.of(3),
				Units.Second.of(10),
				ModifiedSignalLogger.logState()
			),
			new Mechanism(
				(volts) -> this.setVoltage(volts.in(Units.Volt)), 
				null, 
				this,
				"elevator voltage sysid")
		);


		// this.sim = new ElevatorSim(
		// 	DCMotor.getKrakenX60Foc(2), 
		// 	1/ElevatorConstants.kNativeToMeters, 
		// 	Pounds.of(25).in(Kilogram), 
		// 	Units.Inches.of(2).in(Meter), 
		// 	minLength, 
		// 	maxLength, 
		// 	true, 
		// 	minLength, 
		// 	0.05,0.05,0.05);
    }


    public void setHeight(double height) {
        this.setPositionMotionMagic(MathUtil.clamp(height, minLength, maxLength), this.kG);
    }

	public Command elevSysidQuasiFor(){
		return voltageSysid.quasistatic(Direction.kForward);
	}

	public Command elevSysidQuasiBack(){
		return voltageSysid.quasistatic(Direction.kReverse);
	}


	public Command elevSysidDynamFor(){
		return voltageSysid.dynamic(Direction.kForward);
	}

	public Command elevSysidDynamBack(){
		return voltageSysid.dynamic(Direction.kReverse);
	}

	public void setHeightNative(double nativePos){
		this.setPositionMotionMagic(MathUtil.clamp(nativePos * positionConversion, this.minLength, this.maxLength), 0);
	}

	public void setEncoderHeight(Distance height){
		this.setEncoder(height.in(Meter));
	}

	public void setEncoderHeightNative(double rotations){
		this.setEncoderNative(rotations);
	}

    public void setHeight(Distance height) {
        setHeight(height.in(Meters));
    }

    public Distance getHeight() {
        return Meters.of(getPosition());
    }

    public void setOpenLoop(double out) {
        this.setOpenloop(out);
    }

    public boolean heightReached(Distance height, Distance tolerance) {
        return getHeight().minus(height).abs(Meters) < tolerance.in(Meters);
    }

    public double getHeightM() {
        return getPosition();
    }

	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.recordOutput("elevatorgood", getHeightM() >= ElevatorConstants.kClearHeight.in(Meters));
	}

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Elevator";
    }
}
