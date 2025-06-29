package team3647.frc2025.subsystems.pivot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import team3647.frc2025.Util.InverseKinematics;
import team3647.lib.TalonFXSubsystem;

public class PivotReal extends TalonFXSubsystem implements Pivot {

    private final Angle hardMaxAngle;
    private final Angle hardMinAngle;
    private final Angle kClearAngle, kLowClearAngle;

    private final Supplier<Distance> elevatorHeight;
    Trigger ignoreSoftLimits;

    double kG, angle = 0;

    public PivotReal(
            TalonFX master,
            Angle hardMaxAngle,
            Angle hardMinAngle,
            double kG,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            Angle kClearAngle,
            // lowclearangle = max angle when the pivot is blocked by the intake going up
            Angle kLowClearAngle,
            Supplier<Distance> elevatorHeight,
            double kDt,
            Trigger ignoreSoftLimits) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.hardMaxAngle = hardMaxAngle;
        this.hardMinAngle = hardMinAngle;
        this.kClearAngle = kClearAngle;
        this.kLowClearAngle = kLowClearAngle;
        this.elevatorHeight = elevatorHeight;

        this.ignoreSoftLimits = ignoreSoftLimits;

        this.kG = kG;
    }

    public void setAngle(Angle angle) {
        // Current ff = Amps.of(kG * Math.cos(angle.in(Radian)));
        this.angle = angle.in(Radian);
        super.setPositionExpoVoltage(
                MathUtil.clamp(
                        angle.in(Radian), getMinAngle().in(Radian), getMaxAngle().in(Radian)),
                0,
                ignoreSoftLimits.getAsBoolean());
    }

    public void setAngleRads(double angle) {
        // Current ff = Amps.of(kG * Math.cos(angle.in(Radian)));

        super.setPositionExpoVoltage(
                MathUtil.clamp(angle, getMinAngle().in(Radian), getMaxAngle().in(Radian)), 0);
    }

    public Angle getAngle() {
        return Radian.of(getPosition());
    }

    public void setEncoderAngle(Angle angle) {
        this.setEncoder(angle.in(Radian));
    }

    public void setEncoderAngle(double angleRads) {
        this.setEncoder(angleRads);
    }

    public double getAngleRads() {
        return getAngle().in(Radian);
    }

    public double getAngleDegs() {
        return getAngle().in(Degree);
    }

    public void setEncoderNative(double position) {
        super.setEncoderNative(position);
    }

    public Angle getMinAngle() {
        return InverseKinematics.getPivotMin(elevatorHeight.get());
    }

    public Angle getMaxAngle() {
        return InverseKinematics.getPivotMax(elevatorHeight.get());
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleRads() > lowBound && getAngleRads() < highBound;
    }

    public boolean angleReached(Angle angle, Angle tolerance) {
        return getAngle().minus(angle).abs(Radian) < tolerance.in(Radian);
    }

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        super.readPeriodicInputs();
        // Logger.recordOutput("DEBUG/pivot/demand",
        // this.master.getClosedLoopReference().getValueAsDouble() * positionConversion);

    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
