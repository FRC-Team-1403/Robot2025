package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
    
    //Variable for local copy of Wrist subsystem
    private final WristSubsystem m_wrist;
    //Variable for local copy of angle setpoint (Degrees)
    private final double m_angle;

    /***
     * Sets the wrist to a certain angle
     * 
     * @param wrist Wrist subsystem
     * @param angle Angle setpoint (Degrees)
     */
    public WristCommand(WristSubsystem wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        //Moves the wrist
        m_wrist.moveToSetpoint(m_angle);
    }

    @Override
    public boolean isFinished() {
        //Ends the command when the wirst is at the setpoint
        return m_wrist.isAtSetpoint();
    }

}
