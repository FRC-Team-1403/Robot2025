// package team1403.robot.commands;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import org.littletonrobotics.junction.Logger;

// import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

// import edu.wpi.first.wpilibj2.command.Command;
// import team1403.robot.subsystems.WristSubsystem;

// public class WristCommand extends Command {
//     private BooleanSupplier m_zero;
//     private BooleanSupplier m_low;
//     private BooleanSupplier m_mid;
//     private WristSubsystem m_wrist;
    
//     public WristCommand(WristSubsystem wrist, BooleanSupplier zero, BooleanSupplier low, BooleanSupplier mid) {
//         m_wrist = wrist;
//         m_zero = zero;
//         m_low = low;
//         m_mid = mid;

//         addRequirements(m_wrist);
//     }

//     public void init() {}
    
//     @Override
//     public void execute() {
//         if(m_zero.getAsBoolean()) {
//             m_wrist.setWristAngle(.2);
//         }
//         else if(m_low.getAsBoolean())
//             m_wrist.setWristAngle(0);
//         else if(m_mid.getAsBoolean())
//             m_wrist.setWristAngle(-.15);
//     }
// }
