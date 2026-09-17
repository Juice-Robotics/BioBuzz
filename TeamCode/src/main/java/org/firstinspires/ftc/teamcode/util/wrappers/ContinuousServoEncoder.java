package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class ContinuousServoEncoder {
    public AnalogInput encoder;
    boolean reversed;
    public double offset = 0;

    public ContinuousServoEncoder(AnalogInput e, boolean reversed){
        encoder = e;
        this.reversed = reversed;
    }
    public double getPosition(){
        if (reversed){
            return (1-(encoder.getVoltage()/3.3))*360;
        }
        return encoder.getVoltage()/3.3*360;
    }
    public void setPosition(double pos){
        offset = pos - this.getPosition();
    }
}