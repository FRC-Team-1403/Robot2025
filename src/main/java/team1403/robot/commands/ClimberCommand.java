package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    
    private ClimberSubsystem m_climber;
    private BooleanSupplier m_button;

    public ClimberCommand(ClimberSubsystem climber, BooleanSupplier button) {
        m_climber = climber;
        m_button = button;

        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        if (m_button.getAsBoolean()) {
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
