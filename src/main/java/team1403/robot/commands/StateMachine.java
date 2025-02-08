package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.*;

public class StateMachine extends Command {
    private IntakeSubsystem m_intakeSubsystem;
    private WristSubsystem m_wristSubsystem;
    private Elevator m_elevatorSubsystem;
    private static State m_state;
    
    public enum State{
        loading,
        driving,
        aligning,
        placing
    }

    public StateMachine(IntakeSubsystem intake, WristSubsystem wrist, Elevator elevator){
        m_intakeSubsystem = intake;
        m_wristSubsystem = wrist;
        m_elevatorSubsystem = elevator;

        addRequirements(m_intakeSubsystem, m_wristSubsystem, m_elevatorSubsystem);
    }

    public void execute(){
        switch(m_state){
            case loading: {
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.source);
                //TODO align with april tag and add option to move sides
            } case driving: {
                m_elevatorSubsystem.moveToSetpoint(Constants.Elevator.down);
                //TODO control based on elevator buttons
            } case aligning: {
                //TODO add logic for aligning based on coral position (CAN range)
            } case placing: {
                Blackbox.place();
            }
        }
    }
    
}
