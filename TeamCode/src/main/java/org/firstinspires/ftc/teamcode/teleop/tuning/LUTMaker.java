
package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.helpers.AutoShoot;

import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp
//@Disabled
public class LUTMaker extends OpMode{
    public static float HOOD_POS = .1f; // bottom position .6 is top

    public static double flywheelSpeed = 0;
    //DistDataLogger logger;
    Robot robot;
    List<Action> actionsQueue = new ArrayList<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, false);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        robot.shooter.hood.setPosition(HOOD_POS);

        double flywheelRPM = robot.shooter.encoder.motor.getVelocity() * 60 / 28;
        telemetry.addData("vPos ", flywheelRPM);

        robot.pinpoint.update();
        Pose2D pose = robot.pinpoint.getPosition();
        Pose2D vT = new Pose2D(DistanceUnit.INCH, -14.25, -16, AngleUnit.DEGREES,0);
        double dx = vT.getX(DistanceUnit.INCH) - pose.getX(DistanceUnit.INCH);
        double dy = vT.getY(DistanceUnit.INCH) - pose.getY(DistanceUnit.INCH);

        double distance = Math.hypot(dx, dy);

        if (gamepad1.leftBumperWasPressed() && !robot.shooting) {
            actionsQueue.add(robot.shootingAction(flywheelSpeed));
        }
        if (gamepad1.crossWasPressed()) {
            if (robot.intaking) {
                actionsQueue.add(robot.stopIntakeAndTransfer());
            }
        }
        robot.pinpoint.update();
        robot.shooter.update();

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        robot.setDrivePower(x, y, rx);

        List<Action> newActions = new ArrayList<>();
        for (Action action : actionsQueue) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        actionsQueue = newActions;

        telemetry.addData("dist", distance);
        telemetry.addData("flywheel target ", flywheelSpeed);
        telemetry.update();
    }
}
