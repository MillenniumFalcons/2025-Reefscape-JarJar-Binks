package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Radian;

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
                MathUtil.clamp(angle.in(Units.Radian), minAngle.in(Radian), maxAngle.in(Radian)),
                0);
    }

    public Angle getAngle() {
        return Radian.of(getPosition());
    }

    public double getAngleRads() {
        return getAngle().in(Radian);
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleRads() > lowBound && getAngleRads() < highBound;
    }

    public boolean angleReached(double angle, double tolerance) {
        return Math.abs(getAngleRads() - angle) < tolerance;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
