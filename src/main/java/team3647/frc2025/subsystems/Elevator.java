package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;

public class Elevator extends TalonFXSubsystem {

    final double minLength, maxLength, kG;

	public final SysIdRoutine sysid;

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
				Units.Volts.of(0.9).per(Units.Second), 
				Units.Volts.of(3), 
				Units.Seconds.of(10), 
				ModifiedSignalLogger.logState()), 
			new Mechanism(
				(volts) -> setVoltage(volts.in(Units.Volt)), 
				null, 
				this,
				"elevator sysid"));
    }

    public void setHeight(double height) {
        this.setPositionExpoVoltage(MathUtil.clamp(height, minLength, maxLength), this.kG);
    }

	public Command elevSysidQuasiFor(){
		return sysid.quasistatic(Direction.kForward);
	}

	public Command elevSysidQuasiBack(){
		return sysid.quasistatic(Direction.kReverse);
	}


	public Command elevSysidDynamFor(){
		return sysid.dynamic(Direction.kForward);
	}

	public Command elevSysidDynamBack(){
		return sysid.dynamic(Direction.kReverse);
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
    public String getName() {
        // TODO Auto-generated method stub
        return "Elevator";
    }
}
