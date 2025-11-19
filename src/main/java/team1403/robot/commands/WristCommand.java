package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.AlgaeWristSubsystem;

public class WristCommand extends Command {
    private final AlgaeWristSubsystem m_algae;
    private final double angle;

    public WristCommand(AlgaeWristSubsystem wrist, double angle) {
        m_algae = wrist;
        this.angle = angle;
        addRequirements(m_algae);
    }

    @Override
    public void initialize() {
        m_algae.setWristAngle(angle);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_algae.stop();
    }

    @Override
    public boolean isFinished() {

        return m_algae.isAtSetpoint();
    }
}
