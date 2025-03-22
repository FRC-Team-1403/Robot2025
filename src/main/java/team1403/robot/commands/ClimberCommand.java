package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.ClimberSubsystem;


public class ClimberCommand extends Command {

    //Variable for local instantce of Climber subsystem
    public ClimberSubsystem m_climber;
    //Variable for local copy of motor speed
    public double m_speed;

    /**
     * Runs the climber
     * 
     * @param climber the climber subsystem
     * @param speed the speed at which to move the motor
     */
    public ClimberCommand(ClimberSubsystem climber, double speed) {
        m_climber = climber;
        m_speed = speed;
    
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //Sets the speed of the climber motor
        m_climber.setMotorSpeed(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        //Stops the climber motor when the command ends
        m_climber.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}