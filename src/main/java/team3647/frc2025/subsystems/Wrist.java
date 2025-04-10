package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;
import team3647.frc2025.Util.InverseKinematics;
import team3647.frc2025.Util.SuperstructureState;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {

    Angle minAngle, maxAngle;

    private final Supplier<Angle> getPivotAngle;
    private final Supplier<Distance> getElevHeight;

    public Wrist(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            Angle minAngle,
            Angle maxAngle,
            Supplier<Distance> elevHeight,
            Supplier<Angle> pivotAngle,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;

        this.getElevHeight = elevHeight;
        this.getPivotAngle = pivotAngle;
    }

    public void setAngle(Angle angle) {

        setPositionExpoVoltage(
                MathUtil.clamp(
                        angle.in(Units.Degree), minAngle.in(Degree), getMaxAngle().in(Degree)),
                0);
    }

    public void setEncoderAngle(Angle angle) {

        setEncoder(angle.in(Degree));
    }

    public Angle getMaxAngle() {
        return InverseKinematics.getWristOutofTheWayMaxAngle(
                new SuperstructureState(getPivotAngle.get(), getElevHeight.get(), getAngle()),
                maxAngle);
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
        super.periodic();
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
