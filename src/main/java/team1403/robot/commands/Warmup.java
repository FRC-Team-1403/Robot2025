package team1403.robot.commands;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.ClimberSubsystem;
import team1403.robot.subsystems.CoralIntakeSubsystem;
import team1403.robot.subsystems.ElevatorSubsystem;
import team1403.robot.subsystems.LEDSubsystem;
import team1403.robot.subsystems.WristSubsystem;

public class Warmup extends Command{
    
    //private AlgaeIntake m_AlgaeIntake;
    private Blackbox m_Blackbox;
    private ClimberSubsystem m_Climber;
    private CoralIntakeSubsystem m_CoralIntake;
    private ElevatorSubsystem m_Elevator;
    private LEDSubsystem m_LED;
    private WristSubsystem m_Wrist; 

    public Warmup(
        //AlgaeIntake AlgaeIntake, 
        Blackbox Blackbox, 
        ClimberSubsystem Climber, 
        CoralIntakeSubsystem CoralIntake, 
        ElevatorSubsystem Elevator, 
        LEDSubsystem LED, 
        WristSubsystem Wrist) 
        {
            //m_AlgaeIntake = AlgaeIntake;
            m_Blackbox = Blackbox;
            m_Climber = Climber;
            m_CoralIntake = CoralIntake;
            m_Elevator = Elevator;
            m_LED = LED;
            m_Wrist = Wrist;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //TODO Blackbox

        //TODO Climber

        //TODO Coral Intake

        //TODO Elevator

        //TODO LED

        //TODO Wrist

        Commands.print("Robot Warmup Complete!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
