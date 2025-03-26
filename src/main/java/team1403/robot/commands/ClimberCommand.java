package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ClimberSubsystem;


public class ClimberCommand extends Command {

    //Variable for local instantce of Climber subsystem
    public ClimberSubsystem m_climber;
    //Variable for local copy of motor direction
    public boolean isGoingDown;

    /**
     * Runs the climber
     * 
     * @param climber the climber subsystem
     * @param goingDown the direction at which to move the motor
     */
    public ClimberCommand(ClimberSubsystem climber, boolean goingDown) {
        m_climber = climber;
        isGoingDown = goingDown;
    
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {}

    /*
     * clutch engaged -> the button is pressed and the ratcheting is disabled,
     * motor can move both directions
     * 
     * clutch disengaged -> the button is unpressed (default setpoint mechanicaly),
     * motor can only move upwards
     */
    @Override
    public void execute() {
        if(isGoingDown){
            m_climber.setServo(Constants.Climber.clutchEngage);
            if(m_climber.getMotorPosition() > Constants.Climber.downPosition)
                m_climber.setMotorSpeed(Constants.Climber.downSpeed);
        } else {
            m_climber.setServo(Constants.Climber.clutchDisengage);
            if(m_climber.getMotorPosition() < Constants.Climber.upPosition)
                m_climber.setMotorSpeed(Constants.Climber.upSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        //Stops the climber motor when the command ends
        m_climber.stopMotors();
        m_climber.setServo(Constants.Climber.clutchDisengage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}