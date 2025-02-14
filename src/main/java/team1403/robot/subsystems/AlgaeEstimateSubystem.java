package team1403.robot.subsystems;

import team1403.robot.Constants;


public class AlgaeEstimateSubystem {
    int fovX = 0;
    int fovY = 0;
    double k = Constants.Vision.algaeEstimateKonstant;
    double distance = 1;
    
    int bboxX;
    int bboxY;

    public double getDistance() {
        double a = 0;
        return k*(Math.sqrt(1/a));
    }
}



