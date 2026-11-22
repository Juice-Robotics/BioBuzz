
package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.helpers.AutoShoot;


@Config
@TeleOp
//@Disabled
public class PresetTuner extends OpMode{
    private PIDController veloController1;

    public static double vP = 0.007, vI = 0.0, vD = 0, vF = 0.1;

    public static int vTarget = 0;
    public static double HOOD_POS = .1; // bottom position .6 is top
    public static boolean GATE = true; // .3 close .6 open
    public static double INTAKE_POW = 0;

    private DcMotorEx flywheelMotor1;
    private DcMotorEx flywheelMotor2;
    private DcMotorEx flywheelEncoder;
    private Servo hood;

    private DcMotorEx intake;

    private Servo gate;
    public static double GATE_POS = 0.59;

    public static double shooter1Speed = 0;
    double turretSpeed;
    ElapsedTime timer = new ElapsedTime();
    public GoBildaPinpointDriver pinpoint;

    //DistDataLogger logger;


    @Override
    public void init() {
        //MatchStorage.setAllianceColor(AllianceColor.BLUE);
        veloController1 = new PIDController(vP, vI, vD);
//        turretController1 = new PIDController(tP, tI, tD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheelEncoder = hardwareMap.get(DcMotorEx.class, "rightBack");
        flywheelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        timer.reset();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(-8.5, -110, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {

        hood.setPosition(HOOD_POS);
//        intake.setPower(INTAKE_POW);
//        gate.setPosition(0.3);
        if (!GATE) {
            gate.setPosition(0.3);
            intake.setPower(1);
        } else {
            gate.setPosition(GATE_POS);
            intake.setPower(INTAKE_POW);
        }

        veloController1.setPID(vP, vI, vD);
        double flywheelRPM = flywheelEncoder.getVelocity() * 60 / 28;
        telemetry.addData("vPos ", flywheelRPM);
        if (vTarget > 100){
            shooter1Speed = veloController1.calculate(flywheelRPM, vTarget);
            shooter1Speed += vTarget/Math.abs(vTarget) * vF;
        } else {
            shooter1Speed = 0;
        }
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        Pose2D vT = new Pose2D(DistanceUnit.INCH, -14.25, -16, AngleUnit.DEGREES,0);
        double dx = vT.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH);
        double dy = vT.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH);

        double distance = Math.hypot(dx, dy);

        telemetry.addData("dist", distance);

        telemetry.addData("vPow ", shooter1Speed);
        flywheelMotor1.setPower(shooter1Speed);
        flywheelMotor2.setPower(shooter1Speed);

        telemetry.addData("Motor1 Amps",flywheelMotor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor2 Amps",flywheelMotor2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("flywheel target ", vTarget);
        telemetry.update();
    }
}
