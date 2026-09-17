package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

//TODO: AT SOME POINT REFOLDER THIS DIRECTORY not done rn to avoid merge conflicts
public class ContinuousServo extends Component {

    private float speed;
    public CRServo servo;
    public ContinuousServoEncoder encoder;

    public ContinuousServo(int port, String name, HardwareMap map){
        super(port, name);
        servo = map.crservo.get(name);
    }

    public ContinuousServo(int port, String name, HardwareMap map, String encoder, boolean reversed){
        super(port, name);
        servo = map.crservo.get(name);
        this.encoder = new ContinuousServoEncoder(map.get(AnalogInput.class, encoder), reversed);
    }

    public void setSpeed(float speed) {
        this.speed = speed;
        servo.setPower(speed);
    }

    public float getSpeed(){
        return speed;
    }

    public float getAngle() {
        return (float) (encoder.getPosition());
    }

}