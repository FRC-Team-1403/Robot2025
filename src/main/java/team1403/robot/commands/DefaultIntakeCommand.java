package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class DefaultIntakeCommand extends Command {
    
    private CoralIntakeSubsystem m_coralIntake;

    public DefaultIntakeCommand(CoralIntakeSubsystem coralIntake)
    {
        m_coralIntake = coralIntake;

        addRequirements(m_coralIntake);
    }

    public void execute() {
        if(!Blackbox.isCoralLoaded())
            m_coralIntake.setIntakeMotorSpeed(Constants.CoralIntake.intake);
        else
            m_coralIntake.setIntakeMotorSpeed(Constants.CoralIntake.neutral);
    }

    
    public void end(boolean interrupted) {
        m_coralIntake.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}