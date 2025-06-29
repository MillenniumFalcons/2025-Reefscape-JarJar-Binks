// package team3647.frc2025.Util;

// import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import team3647.frc2025.subsystems.superstructure.Superstructure.Branch;
// import team3647.frc2025.subsystems.superstructure.Superstructure.Side;
// import team3647.lib.team6328.VirtualSubsystem;

// import java.util.concurrent.atomic.AtomicInteger;
// import java.util.concurrent.locks.ReentrantLock;
// import java.util.function.Consumer;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.util.sendable.SendableRegistry;

// public class ReefTracker extends VirtualSubsystem {

//         boolean test;
//         ButtonSendable L4Button = new ButtonSendable();

//     public ReefTracker(){
//         super();
//         SmartDashboard.putData(
//             "togggleable button", L4Button
//         );
        
//     }



//     class Stick {
//         Branch branch;
//         Side side;
//         boolean[] isFilled;
//     }
//     class ButtonSendable implements Sendable {
//         private final int m_instance;
//         boolean isFull;
//         private final ReentrantLock m_mutex = new ReentrantLock();
//         private Consumer<Boolean> m_listener = (val) -> {
//             isFull = val;
//         };

//           private static final AtomicInteger s_instances = new AtomicInteger();

//         ButtonSendable(){
//             m_instance = s_instances.getAndIncrement();
//             SendableRegistry.add(this, "SendableChooser", m_instance);
//         }
//         @Override
//         public void initSendable(SendableBuilder builder) {
//             builder.setSmartDashboardType("Boolean Chooser");
//             builder.addBooleanProperty("L4 chooser thing", null, val -> {
//                 m_mutex.lock();
//                 m_listener.accept(val);
//             }
//             );
//         }

//     }

//     @Override
//     public void periodic() {
//        SmartDashboard.putBoolean("PLEASE WORK", L4Button.isFull);
//     }



// }
