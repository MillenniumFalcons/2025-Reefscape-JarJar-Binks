package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {

    Angle minAngle, maxAngle;

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
        setPositionExpoVoltage(
                MathUtil.clamp(angle.in(Units.Degree), minAngle.in(Degree), maxAngle.in(Degree)),
                0);
    }

    public void setEncoderAngle(Angle angle) {
        setEncoder(angle.in(Degree));
    }

    public Angle getAngle() {
        return Degree.of(getPosition());
    }

    public double getAngleDegs() {
        return getPosition();
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleDegs() > lowBound && getAngleDegs() < highBound;
    }

    public boolean angleReached(double angle, double tolerance) {
        return Math.abs(getAngleDegs() - angle) < tolerance;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
