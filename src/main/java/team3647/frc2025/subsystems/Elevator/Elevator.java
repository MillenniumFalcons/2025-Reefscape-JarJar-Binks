package team3647.frc2025.subsystems.Elevator;

import edu.wpi.first.units.measure.Distance;
import team3647.lib.PeriodicSubsystem;

public interface Elevator extends PeriodicSubsystem {
    

    public void setHeight(double hieght);

    public void setHeightNative(double height);

    public void setEncoderHeight(double height);

    public Distance getHeight();

    public boolean heightReached(Distance height, Distance tolerance);

    public void setHeight(Distance height);

    public void setOpenLoop(double output);

}
