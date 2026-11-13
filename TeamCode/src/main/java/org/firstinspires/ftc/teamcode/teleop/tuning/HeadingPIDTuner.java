
package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.hardware.Pinpoint;

@Config
@TeleOp
@Disabled
public class HeadingPIDTuner extends OpMode {
    private PIDController controller1;

    public static double p = 0.02, i = 0.00, d = 0.00045, f=0.05;
    public static double e = 2;
    public static int targetHeading = 0;
    private final double angleSlides = 60;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private VoltageSensor voltageSensor;
    private Pinpoint pinpoint;


    @Override
    public void init() {
        controller1 = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = new Pinpoint(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        double pos = pinpoint.getHeading();
        double pow = controller1.calculate(pos, targetHeading);
        if (Math.abs(pos-targetHeading) > e){
            pow += pow/Math.abs(pow)*f;
        }
        telemetry.addData("pow ", pow);
        telemetry.addData("pos ", pos);

        leftFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);

        telemetry.addData("target ", targetHeading);
        telemetry.update();
    }
}