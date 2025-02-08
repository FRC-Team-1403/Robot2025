package team1403.robot.subsystems;

public class ElevatorSetpoint {

    //radians
    public final double intakeAngle;
    //meters
    public final double elevatorHeight;

    public ElevatorSetpoint(double angle, double height) {
        intakeAngle = angle;
        elevatorHeight = height;
    }
}
