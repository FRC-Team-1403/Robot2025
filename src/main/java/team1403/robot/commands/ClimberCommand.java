// package team1403.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.util.datalog.DataLog;
// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import team1403.robot.subsystems.ClimberSubsystem;


// public class ClimberCommand extends Command {
//     public ClimberSubsystem m_climber;
//     public double m_speed;

//   public ClimberCommand(ClimberSubsystem climber, double speed) {
//     m_climber = climber;
//     m_speed = speed;
    
//     addRequirements(m_climber);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     m_climber.setMotorSpeed(m_speed);
//   }

//   @Override
//   public void end(boolean i) {
//     m_climber.stopMotors();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }