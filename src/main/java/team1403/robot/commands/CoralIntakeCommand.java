package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.ops.DElementCoorBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {

    private BooleanSupplier m_release;
    private BooleanSupplier m_stop;
    private CoralIntakeSubsystem m_intakeSubsystem;
    private int rCounter = 0;
    private int iCounter = 0;
    public CoralIntakeCommand(CoralIntakeSubsystem intakeSubsystem, BooleanSupplier release, BooleanSupplier stop) {
        m_intakeSubsystem = intakeSubsystem;
        m_release = release;
        m_stop = stop;

        addRequirements(m_intakeSubsystem);
    }

    public void init() {}
    
    @Override
    public void execute() {
        if (m_stop.getAsBoolean()) {
            m_intakeSubsystem.setIntakeMotorSpeed(0);
        }
        else if (m_release.getAsBoolean() || rCounter > 0) {
            Constants.CoralIntake.hasPiece = true;
            if (rCounter <= 10) {
                m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.release);
                rCounter++;
            }
            else {
                rCounter = 0;
                Constants.CoralIntake.hasPiece = false;
            }
        }
        else if (Constants.CoralIntake.hasPiece && iCounter > 0) {
            iCounter ++;
            if (iCounter < 50) {
                m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.intake);
            }
            else if ((iCounter >= 50 && iCounter < 100) || (iCounter >= 150 && iCounter < 200) || (iCounter >= 250 && iCounter < 300)) {
                m_intakeSubsystem.setIntakeMotorSpeed(-Constants.CoralIntake.wiggle);
            }
            else if ((iCounter >= 100 && iCounter < 150) || (iCounter >= 200 && iCounter < 250) || (iCounter >= 300 && iCounter < 350)) {
                m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.wiggle);
            }
            else {
                iCounter = 0;
            }
        }
        else if (!Constants.CoralIntake.hasPiece || iCounter > 0) {
                //&& Constants.Elevator.Setpoints.current == Constants.Elevator.Setpoints.source
                //&& Constants.Wrist.Setpoints.current == Constants.Wrist.Setpoints.source) {
            
                m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.intake);
                iCounter = 1;
        }
        else { 
            m_intakeSubsystem.setIntakeMotorSpeed(Constants.CoralIntake.neutral);
        }
    }
}
