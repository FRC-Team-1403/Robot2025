package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import team1403.robot.Constants;
import team1403.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    private ClimberSubsystem m_climber;
    BooleanSupplier m_lift; 
    BooleanSupplier m_lower;

    public ClimberCommand(ClimberSubsystem climber, BooleanSupplier lift, BooleanSupplier lower) {   
        m_climber = climber;
        m_lift = lift; 
        m_lower = lower; 

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.configMotors();
    }

    @Override
    public void execute() {
        if(m_lift.getAsBoolean()){
            m_climber.lift();
        } 
        else if (m_lower.getAsBoolean()) {
            m_climber.lower();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
