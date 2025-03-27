package team1403.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.Constants;
import team1403.robot.subsystems.ClimberSubsystem;


public class ClimberCommand extends Command {

    //Variable for local instantce of Climber subsystem
    private ClimberSubsystem m_climber;
    //Variable for local copy of motor direction
    private boolean isGoingDown;

    /**
     * Runs the climber
     * 
     * @param climber the climber subsystem
     * @param goingDown whether the climber is going down
     */
    public ClimberCommand(ClimberSubsystem climber, boolean goingDown) {
        m_climber = climber;
        isGoingDown = goingDown;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        if(isGoingDown){
            m_climber.setServo(Constants.Climber.ratchetDisengage);
            Commands.waitSeconds(2);
        }
        // m_timer.reset();
    }

    /*
     * ratchet engaged -> the button is ubpressed (default setpoint physically),
     * and the motor can only move upwards
     * 
     * ratched disengaged -> the button is pressed in by the servo,
     * motor can move in both directions
     */
    @Override
    public void execute() {
        if(isGoingDown){
            m_climber.setServo(Constants.Climber.ratchetDisengage);
            if(m_climber.getMotorPosition() > Constants.Climber.downPosition)
                m_climber.setMotorSpeed(Constants.Climber.downSpeed);
        } else {
            m_climber.setServo(Constants.Climber.ratchetEngage);
            if(m_climber.getMotorPosition() < Constants.Climber.upPosition)
                m_climber.setMotorSpeed(Constants.Climber.upSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        //Stops the climber motor when the command ends
        m_climber.stopMotors();
        m_climber.setServo(Constants.Climber.ratchetEngage);
    }

    @Override
    public boolean isFinished() {
        if(isGoingDown)
            return m_climber.getMotorPosition() <= Constants.Climber.downPosition;
        else
            return m_climber.getMotorPosition() >= Constants.Climber.upPosition;
    }
}