package team3647.frc2025.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import team3647.lib.PeriodicSubsystem;

public interface Pivot extends PeriodicSubsystem {

    public void setAngle(Angle angle);

    public void setAngleRads(double angleRads);

    public Angle getAngle();

    public double getAngleRads();

    public double getAngleDegs();

    public void setEncoderAngle(Angle angle);

    public void setEncoderAngle(double angleRads);

    public boolean angleReached(Angle angle, Angle tolerance);

    public boolean angleWithin(double lowBound, double highBound);

    public boolean needToClearElevator();

    public Angle getMaxAngle();

    public Angle getMinAngle();
}
