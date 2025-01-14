package team3647.frc2025.subsystems;

import javax.crypto.KeyGenerator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import team3647.lib.TalonFXSubsystem;

public class Elevator extends TalonFXSubsystem {

   
    double minLength, maxLength;

    public Elevator(TalonFX master,
    TalonFX follower,
    double velocityConversion,
    double positionConversion,
    double nominalVoltage,
    double minLength,
    double maxLength,
    double kDt){
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        super.addFollower(follower, false);
        this.minLength = minLength;
        this.maxLength = maxLength;
    }


    public void setHeight(double height){
        this.setPositionExpoVoltage(MathUtil.clamp(height, minLength, maxLength), 0);
    }

    public double getHeight(){
        return getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Elevator";
    }

}
