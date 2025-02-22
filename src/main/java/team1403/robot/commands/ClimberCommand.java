// package team1403.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.Command;

// import team1403.robot.Constants;
// import team1403.robot.subsystems.ClimberSubsystem;

// public class ClimberCommand extends Command{
//     private ClimberSubsystem m_climber;
//     BooleanSupplier m_lift; 
//     BooleanSupplier m_lower;
//     // double m_speed;

//     public ClimberCommand(ClimberSubsystem climber, BooleanSupplier lift, BooleanSupplier lower, double speed) {   
//         m_climber = climber;
//         m_lift = lift; 
//         m_lower = lower; 
//         m_speed = speed;

//         addRequirements(m_climber);
//     }

//     @Override
//     public void initialize() {
//         m_climber.configMotors();
//     }

//     @Override
//     public void execute() {
//         if(m_lift.getAsBoolean()){
//             m_climber.lift(m_speed);
//         } else if (m_lower.getAsBoolean()) {
//             m_climber.lower(m_speed); //use 
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
