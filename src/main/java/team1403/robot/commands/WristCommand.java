package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
    

    private final WristSubsystem m_wrist;
    private final double m_angle;

    //angle in degrees
    public WristCommand(WristSubsystem wrist, double angle) {
        m_wrist = wrist;
        m_angle = angle / 360.;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.setWristAngle(m_angle);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isAtSetpoint();
    }

}
