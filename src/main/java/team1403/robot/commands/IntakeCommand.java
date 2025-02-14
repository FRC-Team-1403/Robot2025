package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem m_intake;
    private BooleanSupplier m_stop;
    private BooleanSupplier m_start;

    public IntakeCommand(IntakeSubsystem intake, BooleanSupplier stop, BooleanSupplier start) {   
        m_intake = intake;
        m_stop = stop;
        m_start = start;

        addRequirements(m_intake);   
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //m_intake.setIntakeMotorSpeed(m_speed);
        if (m_stop.getAsBoolean()) {
            m_intake.setIntakeMotorSpeed(0);
        } 
        else if (m_start.getAsBoolean()) {
            m_intake.setIntakeMotorSpeed(0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
