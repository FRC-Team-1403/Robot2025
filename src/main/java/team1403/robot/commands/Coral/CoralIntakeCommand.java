package team1403.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Coral_Intake;


// 15.875 inches is the distance 

public class CoralIntakeCommand extends Command{
    Coral_Intake m_subsystem;

    public CoralIntakeCommand(Coral_Intake subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


    public void initialize() {
    }

    public void execute() {
        m_subsystem.setMotorSpeed(0.3);
    } 

    public void end(boolean interrupted) {
        m_subsystem.setMotorSpeed(0);
    }

    public boolean isFinished() {
        return m_subsystem.isIntakeLoaded();
    }
}
