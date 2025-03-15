package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final Angle kClearAngle, kLowClearAngle;

    private final Supplier<Distance> elevatorHeight;

    double kG;

	

    public Pivot(
            TalonFX master,
            Angle maxAngle,
            Angle minAngle,
            double kG,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            Angle kClearAngle,
            // lowclearangle = max angle when the pivot is blocked by the intake going up
            Angle kLowClearAngle,
            Supplier<Distance> elevatorHeight,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.kClearAngle = kClearAngle;
        this.kLowClearAngle = kLowClearAngle;
        this.elevatorHeight = elevatorHeight;

        this.kG = kG;
    }

    public void setAngle(Angle angle) {
        // Current ff = Amps.of(kG * Math.cos(angle.in(Radian)));

        super.setPositionExpoVoltage(
                MathUtil.clamp(
                        angle.in(Radian), getMinAngle().in(Radian), getMaxAngle().in(Radian)),
                0);
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

    public double getAngleRads() {
        return getAngle().in(Radian);
    }

    public double getAngleDegs() {
        return getAngle().in(Degree);
    }

    public void setEncoderNative(double position) {
        super.setEncoderNative(position);
    }

    public boolean needToClearElevator() {
        return getPosition() < kClearAngle.in(Radian)
                && elevatorHeight.get().lt(ElevatorConstants.kClearHeight);
    }

    public Angle getMinAngle() {
        return minAngle;
    }

    public Angle getMaxAngle() {
        return maxAngle;
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
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        super.readPeriodicInputs();
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
