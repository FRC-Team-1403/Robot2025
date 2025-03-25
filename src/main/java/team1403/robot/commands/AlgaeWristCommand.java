package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.AlgaeWristSubsystem;
import team1403.robot.subsystems.Blackbox;

public class AlgaeWristCommand extends Command {
    private AlgaeWristSubsystem m_algaeWrist;
    private double target;

    public AlgaeWristCommand(AlgaeWristSubsystem algaeWristSubsystem, double setpoint) {
        m_algaeWrist = algaeWristSubsystem;
        target = setpoint;
        
        addRequirements(m_algaeWrist);
    }
    
    @Override
    public void initialize() { }

    @Override
    public void execute() {
        m_algaeWrist.setWristAngle(target);
    }
    
    @Override
    public void end(boolean interrupted) {
        if(m_algaeWrist.getWristSetpoint() == Constants.AlgaeWrist.intakingPosition)
            Blackbox.setAlgaeIntaking(true);
        else Blackbox.setAlgaeIntaking(false);
    }

    @Override
    public boolean isFinished() {
        return m_algaeWrist.isAtSetpoint();
    }
}
