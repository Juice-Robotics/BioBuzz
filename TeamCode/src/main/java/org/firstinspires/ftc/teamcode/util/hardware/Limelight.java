package org.firstinspires.ftc.teamcode.util.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight {
    public Limelight3A limelight;
    double turretRadius = 3;// TODO: inches
    LLResult result;
    LLResult lastResult;

    public Limelight(Limelight3A ll) {
        limelight = ll;
    }

    public void start(){
        limelight.start();
    }
    public LLResult getLatestResult(){
        return limelight.getLatestResult();
    }
}