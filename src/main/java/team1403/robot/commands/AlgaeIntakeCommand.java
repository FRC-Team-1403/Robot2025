package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
    
    private AlgaeIntakeSubsystem m_algaeIntake;
    private double m_setpoint;

    public AlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntake, double setpoint) {
        m_algaeIntake = algaeIntake;
        m_setpoint = setpoint;

        addRequirements(m_algaeIntake);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
