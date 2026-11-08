
package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class VeloPIDTuner extends OpMode{
    private PIDController controller1;

    public static double p = 0.006, i = 0.05, d = 0.000;
    public static double f = 0;

    public static int targetRPM = 0;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private VoltageSensor voltageSensor;
    private Servo kicker;

    private DcMotorEx intake;
    private com.qualcomm.robotcore.hardware.CRServo transfer1;
    private com.qualcomm.robotcore.hardware.CRServo transfer2;

    private Servo gate;

    boolean kicked;

    double shooter1Speed;

    boolean oldSquare = false;
    boolean oldRBumper = false;
    boolean isKicked = false;

    boolean timed = false;
    ElapsedTime timer = new ElapsedTime();
    public static double timesd = 0;
    boolean starttime = false;

    int intaking = 0;
    @Override
    public void init() {
        controller1 = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");
        timer.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.square && !oldSquare){
            if (intaking == 0){
                gate.setPosition(0.36);
                intake.setPower(0.8);
                intaking = 2;
            } else if (intaking ==1){
                intake.setPower(0);
                intaking = 0;
                gate.setPosition(0.75);
            } else if (intaking == 2){
                intake.setPower(0.6);
                intaking = 1;
            }

        }
        oldSquare = gamepad1.square;

        controller1.setPID(p, i, d);
        double slides1Pos = motor1.getVelocity();
        telemetry.addData("pos ", slides1Pos);
        if (targetRPM > 0){
            shooter1Speed = controller1.calculate(motor1.getVelocity(), targetRPM) + f;
        } else {
            shooter1Speed = controller1.calculate(motor1.getVelocity(), targetRPM) ;
        }

        telemetry.addData("pow ", shooter1Speed);

        motor1.setPower(shooter1Speed);
        motor2.setPower(shooter1Speed);


        telemetry.addData("timer", timesd);
        telemetry.addData("target ", targetRPM);
        telemetry.update();
    }
}