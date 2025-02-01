package team1403.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.EndEffectorSubsystem;


// 15.875 inches is the distance 

public class IntakeCommand extends Command{
    private EndEffectorSubsystem m_subsystem;

    public IntakeCommand(EndEffectorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


    public void initialize() {
    }

    public void execute() {
        m_subsystem.setIntakeMotorSpeed(0.3);
    } 

    public void end(boolean interrupted) {
        m_subsystem.setIntakeMotorSpeed(0);
        while (m_subsystem.getEncoderValue() != 0) {
            if (m_subsystem.getEncoderValue() > 0) {
                m_subsystem.setWristMotorSpeed(-0.3);
            } else if (m_subsystem.getEncoderValue() < 0) {
                m_subsystem.setWristMotorSpeed(0.3);
            }
        }
        // Wrist start position is at 90???
    }

    public boolean isFinished() {
        return m_subsystem.isIntakeLoaded();
    }
}
