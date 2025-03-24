package team1403.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import team1403.robot.commands.StateMachine;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.CoralIntakeSubsystem;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.datalog.BooleanLogEntry;

public class Logging extends CoralIntakeSubsystem {
    Timer log_Timer = new Timer();

    public void startTime() {
        log_Timer.start();
    }

    public double getTimeValue() {
        return log_Timer.get();
    }

    public double cycleTime() {
        double coralIn = 0;
        double coralOut = 0;
        // while (Blackbox.isCoralLoaded()) {
        // coralIn = log_Timer.get();
        // }
        // double coralOut = log_Timer.get();
        // return (coralOut - corAlIn);
        if (Blackbox.isCoralLoaded()) {
            coralIn = log_Timer.get();
            if (!Blackbox.isCoralLoaded()) {
                coralOut = log_Timer.get();
            }
        }
        return (coralOut - coralIn);

    }
   
    @ Override
    public void periodic() {
        Logger.recordOutput("Coral - Cycle Time", cycleTime());
    }

}
