package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    Angle maxAngle;
    Angle minAngle;

    double kG;

    public Pivot(
            TalonFX master,
            Angle maxAngle,
            Angle minAngle,
            double kG,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;

        this.kG = kG;
    }

    public void setAngle(Angle angle) {
        Current ff = Amps.of(kG * Math.cos(angle.in(Radian)));

        super.setPositionExpoVoltage(
                MathUtil.clamp(angle.in(Radian), minAngle.in(Radian), maxAngle.in(Radian)),
                ff.in(Amps));
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

    public boolean angleReached(Angle angle, Angle tolerance) {
        return getAngle().minus(angle).abs(Radian) < tolerance.in(Radian);
    }

    public void setOpenLoop(double out) {
        this.setOpenloop(out);
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
