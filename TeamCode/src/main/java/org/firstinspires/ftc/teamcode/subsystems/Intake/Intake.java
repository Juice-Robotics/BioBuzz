package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.wrappers.Motor;

public class Intake {
    public Motor motor;
    public Servo leftWedge;
    public Servo rightWedge;


    double intakePower = 1.0;
    double intakeReducedPower = 0.525;
    public double currentPower = 0;

    // Wedge servo positions (0.0 to 1.0)
    // Adjust these values once mounted on the robot frame
    public static double WEDGE_DOWN_LEFT = 0.2;
    public static double WEDGE_DOWN_RIGHT = 0.8;
    public static double WEDGE_UP_LEFT = 0.7;
    public static double WEDGE_UP_RIGHT = 0.3;

    // State tracker: 0 = Off, 1 = Running
    int intakingState = 0;

    public Intake(Motor m, Servo leftWedge, Servo rightWedge) {
        this.motor = m;
        this.leftWedge = leftWedge;
        this.rightWedge = rightWedge;

        this.motor.motor.setCurrentAlert(8.2, CurrentUnit.AMPS);
    }


    public void setPower(double power) {
        motor.setPower(power);
        currentPower = power;
    }

    public void start() {
        setPower(intakePower);
        intakingState = 1;
    }

    public void reverse() {
        setPower(-1.0);
        intakingState = 1;
    }

    public void stop() {
        setPower(0);
        intakingState = 0;
    }

    public void reducedIntake() {
        setPower(intakeReducedPower);
    }

    public double getMotorPower() {
        return motor.motor.getPower();
    }


    public void setWedgesDown() {
        leftWedge.setPosition(WEDGE_DOWN_LEFT);
        rightWedge.setPosition(WEDGE_DOWN_RIGHT);
    }

    public void setWedgesUp() {
        leftWedge.setPosition(WEDGE_UP_LEFT);
        rightWedge.setPosition(WEDGE_UP_RIGHT);
    }
}