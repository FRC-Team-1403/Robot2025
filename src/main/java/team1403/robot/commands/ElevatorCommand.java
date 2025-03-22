package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    //Variable for local copy of Elevator subsystem
    private final ElevatorSubsystem m_elevator;
    //Variable for local copy of Elevator position
    private final double m_pos;

    public ElevatorCommand(ElevatorSubsystem elevator, double position) {
        m_elevator = elevator;
        m_pos = position;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //Move the elevator
        m_elevator.moveToSetpoint(m_pos);
    }

    @Override
    public boolean isFinished() {
        //Ends command when the elevator reaches the setpoint
        return m_elevator.isAtSetpoint();
    } 

}
