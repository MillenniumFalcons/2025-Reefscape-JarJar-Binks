package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Meters;

import javax.crypto.KeyGenerator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import team3647.lib.TalonFXSubsystem;

public class Elevator extends TalonFXSubsystem {

   
    final double minLength, maxLength, kG;

    public Elevator(TalonFX master,
    TalonFX follower,
    double velocityConversion,
    double positionConversion,
    double nominalVoltage,
	double kG,
    double minLength,
    double maxLength,
    double kDt){
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        super.addFollower(follower, false);
        this.minLength = minLength;
        this.maxLength = maxLength;
		this.kG = kG;
    }


    public void setHeight(double height){
        this.setPositionExpoVoltage(MathUtil.clamp(height, minLength, maxLength), this.kG);
    }

	public void setHeight(Distance height){
		setHeight(height.in(Meters));
	}

	public Distance getHeight(){
		return Meters.of(getPosition());
	}

	public void setOpenLoop(double out){
		this.setOpenloop(out);
	}

	public boolean heightReached(Distance height, Distance tolerance){
		return getHeight().minus(height).abs(Meters) < tolerance.in(Meters);
	}

    public double getHeightM(){
        return getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Elevator";
    }

}
