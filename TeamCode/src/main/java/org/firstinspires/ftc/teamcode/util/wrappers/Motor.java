package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Motor extends Component {
    private boolean reverse;
    private double power;
    public DcMotorEx motor;

    public Motor(int port, String name, HardwareMap map, boolean reverse){
        super(port, name);
        this.reverse = reverse;
        this.power = 0;
        motor = map.get(DcMotorEx.class, name);

        if(reverse){
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    private double lastPower = 0;
    public void setPower(double power) {
        this.power = power;
        if (Math.abs(power) < .01) {
            power = 0;
        }
        if (Math.abs(power - lastPower) > 0.03) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public float getEncoderValue(){
        return motor.getCurrentPosition();
    }

    public void setTarget(int ticks){
        motor.setTargetPosition(ticks);
    }

    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getCurrentPos() {
        return motor.getCurrentPosition();
    }

}