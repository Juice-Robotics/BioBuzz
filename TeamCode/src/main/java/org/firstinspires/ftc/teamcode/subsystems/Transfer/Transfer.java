package org.firstinspires.ftc.teamcode.subsystems.Transfer;

import org.firstinspires.ftc.teamcode.util.wrappers.Motor;
public class Transfer {
    public Motor motor;

    double transferPower = 1;
    public double currentPower = 0;

    public Transfer(Motor m){
        motor = m;
    }

    public void setPower(double power){
        motor.setPower(power);
        currentPower = power;
    }

    public void start(){
        setPower(transferPower);
    }

    public void reverse(){
        setPower(-transferPower);
    }

    public void stop(){
        setPower(0);
    }
}
