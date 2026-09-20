package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.wrappers.Motor;
import org.firstinspires.ftc.teamcode.util.wrappers.StepperServo;

public class Shooter {

    public Motor motor1;
    public Motor motor2;
    public Motor encoder;
    public StepperServo gate;

    private boolean shooterEnabled = false;
    private boolean gateOpen = false;

    public double targetRPM = 0;
    public double currentRPM = 0;

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double kF = 0.0;

    private final PIDController shooterPID =
            new PIDController(KP, KI, KD);

    public static float GATE_CLOSED = 0.31f;
    public static float GATE_OPEN = 0.61f;

    public static double TICKS_PER_REV = 28.0;

    public static double RPM_TOLERANCE = 75.0;


    public Shooter(Motor motor1, Motor motor2, Motor encoder,
                   StepperServo gate) {

        this.motor1 = motor1;
        this.motor2 = motor2;
        this.encoder = encoder;
        this.gate = gate;

        motor1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeGate();
    }

    public void update() {

        refreshRPM();

        if (!shooterEnabled) {
            motor1.setPower(0);
            motor2.setPower(0);
            return;
        }

        shooterPID.setPID(KP, KI, KD);

        double shooterPower =
                shooterPID.calculate(getRPM(), targetRPM);

        shooterPower += Math.signum(targetRPM) * kF;

        shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));

        motor1.setPower(shooterPower);
        motor2.setPower(shooterPower);
    }

    public void setRPM(double rpm) {

        targetRPM = rpm;

        shooterEnabled = rpm > 100;
    }

    public void start(double rpm) {
        setRPM(rpm);
    }


    public void stop() {

        shooterEnabled = false;
        targetRPM = 0;

        motor1.setPower(0);
        motor2.setPower(0);
    }
    public void refreshRPM() {

        currentRPM =
                -encoder.motor.getVelocity()
                        * 60.0
                        / TICKS_PER_REV;
    }

    public double getRPM() {
        return currentRPM;
    }

    public boolean isAtSpeed() {

        return shooterEnabled
                && Math.abs(currentRPM - targetRPM)
                <= RPM_TOLERANCE;
    }

    public boolean isEnabled() {
        return shooterEnabled;
    }

    public void closeGate() {

        gate.setPosition(GATE_CLOSED);
        gateOpen = false;
    }

    public void openGate() {

        gate.setPosition(GATE_OPEN);
        gateOpen = true;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }
}