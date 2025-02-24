package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private final ElevatorSubsystem m_elevator;
    private final double m_pos;

    public ElevatorCommand(ElevatorSubsystem elevator, double position) {
        m_elevator = elevator;
        m_pos = position;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.moveToSetPoint(m_pos);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isAtSetpoint();
    } 

}
