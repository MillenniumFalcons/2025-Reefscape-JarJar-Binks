package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import team3647.frc2025.constants.WristConstants;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {

    Angle minAngle, maxAngle;
	double angle = 0;

    public Wrist(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            Angle minAngle,
            Angle maxAngle,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }

    public void setAngle(Angle angle) {
		this.angle = angle.in(Degree);
        setPositionExpoVoltage(
                MathUtil.clamp(angle.in(Units.Degree), minAngle.in(Degree), maxAngle.in(Degree)),
                0);
    }

    public void setEncoderAngle(Angle angle) {
		this.angle =angle.in(Degree);
        setEncoder(angle.in(Degree));
    }

    public Angle getAngle() {
        return Degree.of(angle);
    }

    public double getAngleDegs() {
        return angle;
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleDegs() > lowBound && getAngleDegs() < highBound;
    }

    public boolean angleReached(double angle, double tolerance) {
        return Math.abs(getAngleDegs() - angle) < tolerance;
    }

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		super.periodic();
		Logger.recordOutput("IK/wristAngle", getAngle().in(Degree));
	}

    @Override
    public String getName() {
        return "Wrist";
    }
}
