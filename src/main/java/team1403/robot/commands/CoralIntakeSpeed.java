package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeSpeed extends Command {
    
    //Variable for local instance of Coral subsystem
    private CoralIntakeSubsystem m_coralIntake;
    //Varaible for local copy of the speed to run the motor
    private double m_speed;

    /**
     * Spins the intake wheels
     * 
     * @param coralIntake Coral subsystem instance
     * @param speed how fast to spin the wheels
     */
    public CoralIntakeSpeed(CoralIntakeSubsystem coralIntake, double speed)
    {
        m_coralIntake = coralIntake;
        m_speed = speed;

        addRequirements(m_coralIntake);
    }

    public void execute() {
        //Sets the speed of the motor
        m_coralIntake.setIntakeMotorSpeed(m_speed);
    }

    
    public void end(boolean interrupted) {
        //Sets the speed of the motor to 0 when the command ends
        m_coralIntake.setIntakeMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}