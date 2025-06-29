package team3647.frc2025.subsystems.wrist;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import team3647.lib.TalonFXSubsystem;

public class WristReal extends TalonFXSubsystem implements Wrist {

    Angle minAngle, maxAngle;

    public WristReal(
            TalonFX master,
            Angle maxAngle,
            Angle minAngle,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        super(master, positionConversion, velocityConversion, nominalVoltage, kDt);
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
    public void periodic() {
        // TODO Auto-generated method stub
        periodic();
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
