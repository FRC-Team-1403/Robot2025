package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private BooleanSupplier m_release;
    private CoralIntakeSubsystem m_intakeSubsystem;
    private int counter;
    
    public CoralIntakeCommand(CoralIntakeSubsystem intakeSubsystem, BooleanSupplier release) {
        m_intakeSubsystem = intakeSubsystem;
        m_release = release;

        addRequirements(m_intakeSubsystem);
    }

    public void init() {}
    
    @Override
    public void execute() {
        if (m_release.getAsBoolean() || counter > 0) {
            Constants.CoralIntake.hasPiece = true;
            if (counter <= 50) {
                m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.release);
                counter++;
            }
            else {
                counter = 0;
                Constants.CoralIntake.hasPiece = false;
            }
        }
        else if (!Constants.CoralIntake.hasPiece) {
                //&& Constants.Elevator.Setpoints.current == Constants.Elevator.Setpoints.source
                //&& Constants.Wrist.Setpoints.current == Constants.Wrist.Setpoints.source) {
            m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.intake);
        }
        else { 
            m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.neutral);
        }
    }
}
