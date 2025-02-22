package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeSpeed extends Command {
    
    private CoralIntakeSubsystem m_subsystem;
    private double m_speed;

    public CoralIntakeSpeed(CoralIntakeSubsystem s, double speed)
    {
        m_subsystem = s;
        m_speed = speed;

        addRequirements(m_subsystem);
    }

    public void execute() {
        m_subsystem.setIntakeMotorSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
