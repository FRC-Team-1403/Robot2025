package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.AlgaeIntakeSubsystem;
import team1403.robot.subsystems.Blackbox;

public class DefaultAlgaeIntakeCommand extends Command {
    private AlgaeIntakeSubsystem m_algaeIntake;

    public DefaultAlgaeIntakeCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        m_algaeIntake = algaeIntakeSubsystem;

        addRequirements(m_algaeIntake);
    }

    public void initialize(){
        m_algaeIntake.stop();
    }

    public void execute() {
        if(Blackbox.isAlgaeIntaking()) {
            
            m_algaeIntake.setIntakeSpeed(Constants.AlgaeIntake.intakeSpeed);
        }

    }

    public boolean isFinished() {
        return false;
    }
}
