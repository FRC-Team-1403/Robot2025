package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.ClimberSubsystem;
import team1403.robot.Constants;

public class ClimberCommand {
    private Climber m_climber;
    double m_speed;

    public ClimberCommand(Climber climber, double speed) {   
        m_climber = climber;
        m_speed = speed;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_climber.setSpeed(m_speed);      
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
