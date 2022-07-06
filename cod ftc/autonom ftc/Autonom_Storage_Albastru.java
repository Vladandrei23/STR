package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.ContourPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Autonom Storage Albastru", group="Auto")
///@Disabled
public class Autonom_Storage_Albastru extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private int minRectangleArea = 0;

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    double preload = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor carusel = hardwareMap.dcMotor.get("carusel");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        Servo kicker = hardwareMap.servo.get("kicker");

        glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera.setTargetPosition(0);
        glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setTargetPosition(0);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glisiera.setDirection(DcMotor.Direction.FORWARD);

        drive.setPoseEstimate(new Pose2d(-33.5, 63, Math.toRadians(270)));

        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-39, 54, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(270)))
                .build();

        TrajectorySequence myTrajectory12 = drive.trajectorySequenceBuilder(myTrajectory.end())
                .lineToLinearHeading(new Pose2d(-62, 23, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-46, 23, Math.toRadians(0)))
                .build();

        TrajectorySequence myTrajectory2 = drive.trajectorySequenceBuilder(myTrajectory12.end())
                .lineToLinearHeading(new Pose2d(-62, 23, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-68, 35.5, Math.toRadians(0)))
                .build();

        Trajectory fataputin = drive.trajectoryBuilder(myTrajectory12.end())
                .forward(5)
                .build();

        Trajectory spateputin = drive.trajectoryBuilder(fataputin.end())
                .back(5)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        waitForStart();

        Thread closeCameraThread = new Thread(() -> webcam.closeCameraDevice());
        closeCameraThread.start();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            double rectangleArea = pipeline.getRectArea();

            if (rectangleArea >= minRectangleArea) {
                if (pipeline.getRectMidpointX() < 220) {
                    telemetry.addData("TSE", "Low");
                    preload = 3;
                } else if (pipeline.getRectMidpointX( ) >= 220 && pipeline.getRectMidpointX() < 440) { // * pipeline.getRectWidth()
                    telemetry.addData("TSE", "Mid");
                    preload = 2;
                } else if (pipeline.getRectMidpointX() >= 440) {
                    telemetry.addData("TSE", "High");
                    preload = 1;
                }
            }
            telemetry.update();

            kicker.setPosition(0.67);
            drive.followTrajectorySequence(myTrajectory);
            carusel.setPower(0.5);
            sleep(2500);
            carusel.setPower(0);
            drive.followTrajectorySequence(myTrajectory12);
            if (preload==1) {
                glisiera.setTargetPosition(294);
                glisiera.setPower(0.4);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(134);
                brat.setPower(0.7);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(728);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(1540);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(2025);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                kicker.setPosition(1);
                glisiera.setTargetPosition(728);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(134);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(294);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(0);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(0);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                kicker.setPosition(0.8);
                drive.followTrajectorySequence(myTrajectory2);
            }
            if (preload==2) {
                glisiera.setTargetPosition(294);
                glisiera.setPower(0.4);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(134);
                brat.setPower(0.7);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(728);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(1725);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                drive.followTrajectory(fataputin);
                kicker.setPosition(1);
                drive.followTrajectory(spateputin);
                brat.setTargetPosition(134);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(294);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(0);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(0);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                kicker.setPosition(0.8);
                drive.followTrajectorySequence(myTrajectory2);
            }
            if (preload==3) {
                glisiera.setTargetPosition(294);
                glisiera.setPower(0.4);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(134);
                brat.setPower(0.7);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(728);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(2022);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                drive.followTrajectory(fataputin);
                kicker.setPosition(1);
                drive.followTrajectory(spateputin);
                brat.setTargetPosition(134);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(294);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                brat.setTargetPosition(0);
                while (brat.isBusy() && opModeIsActive()) {
                    idle();
                }
                glisiera.setTargetPosition(0);
                while (glisiera.isBusy() && opModeIsActive()) {
                    idle();
                }
                kicker.setPosition(0.8);
                drive.followTrajectorySequence(myTrajectory2);
            }
        }
    }
}
