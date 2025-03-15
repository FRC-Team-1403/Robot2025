// package team1403.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import team1403.robot.Constants;
// import team1403.robot.subsystems.AlgaeIntakeSubsystem;

// public class AlgaeIntakeCommand extends Command {
    
//     private AlgaeIntakeSubsystem m_algaeIntake;
//     private double m_setpoint;
//     private double m_speed;

//     public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntake, double setpoint, double speed) {
//         m_algaeIntake = algaeIntake;
//         m_setpoint = setpoint;
//         m_speed = speed;

//         addRequirements(m_algaeIntake);
//     }

//     @Override
//     public void execute() {
//         if (Math.abs(m_algaeIntake.getWristAngle() - m_setpoint) > 5) {
//             m_algaeIntake.setWristAngle(m_setpoint);
//         }

//         if (m_setpoint == Constants.AlgaeIntake.downPos || m_algaeIntake.hasAlgae()) {
//             m_algaeIntake.setIntakeSpeed(0);
//         }
//         else {
//             m_algaeIntake.setIntakeSpeed(m_speed);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
