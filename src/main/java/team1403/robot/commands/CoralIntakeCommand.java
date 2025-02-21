package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private BooleanSupplier m_release;
    private IntakeSubsystem m_intakeSubsystem;
    private int counter;
    
    public CoralIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier release) {
        m_intakeSubsystem = intakeSubsystem;
        m_release = release;

        addRequirements(m_intakeSubsystem);
    }

    public void init() {}
    
    @Override
    public void execute() {
        if (m_release.getAsBoolean() || counter > 0) {
            if (counter < 5) {
                m_intakeSubsystem.setIntakeMotorSpeed(-0.3);
                counter++;
            }
            else {
                counter = 0;
            }
        }
        else if (!m_intakeSubsystem.hasPiece() 
                && Constants.Elevator.Setpoints.current == Constants.Elevator.Setpoints.source
                && Constants.Wrist.Setpoints.current == Constants.Wrist.Setpoints.source) {
            m_intakeSubsystem.setIntakeMotorSpeed(0.2);
        }
        else { 
            m_intakeSubsystem.setIntakeMotorSpeed(0.02);
        }
    }
}
