package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeSpeed extends Command {
    
    private CoralIntakeSubsystem m_coralIntake;
    private double m_speed;

    public CoralIntakeSpeed(CoralIntakeSubsystem coralIntake, double speed)
    {
        m_coralIntake = coralIntake;
        m_speed = speed;

        addRequirements(m_coralIntake);
    }

    public void execute() {
        m_coralIntake.setIntakeMotorSpeed(m_speed);
    }

    
    public void end(boolean interrupted) {
        m_coralIntake.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}