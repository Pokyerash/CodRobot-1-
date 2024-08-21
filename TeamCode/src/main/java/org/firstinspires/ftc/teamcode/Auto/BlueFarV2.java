package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.ColorVisionProcessor;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionBlueFar;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedFar;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name ="BlueFarV2", group = "CENTERSTAGE")

public class BlueFarV2 extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(39.2, -64, Math.toRadians(90));

    Trajectory pos1;
    Trajectory pos2;
    Trajectory pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory posback;
    Trajectory pos8;

    TrajectorySequence pos3right;

    Trajectory pos10;
    Trajectory pos9;
    Trajectory pos11;
    Trajectory pos12;





    Trajectory posinspate;
    Trajectory posint;
    Trajectory posplace;
    Trajectory postras;
    Trajectory pospixeli;

    CASE Case;

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    enum Location{
        Left,
        Center,
        Right
    }




    public enum CASE {
        left,
        center,
        right
    }

    int cazzz=2;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    PropDetectionBlueFar processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        teleop.airplane.initAirplane(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        processor = new PropDetectionBlueFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();


     /*   pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(53, -38.12, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(32.4, -47.2, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3right= drive.trajectorySequenceBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(-35, -11.8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        //to pannel
        pos4 = drive.trajectoryBuilder(pos3right.end())
                .splineToConstantHeading(new Vector2d(-26, -16.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-50.7, -24.51), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -10), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, -3.14), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(61.387, -3.4),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(30, 0),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, 0),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.65,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.8, -29), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -16), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })
                .addTemporalMarker(1.4,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.4);
                    while(teleop.intake.intakeUAD.getPosition()==0.4) {
                        collect();
                    }

                })
                .addTemporalMarker(2.5,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, -10.9), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(61.67, -9.43),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(30, -9.25),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -9.25),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.7, -39.5), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(-40, -36),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    teleop.lift.Retract();
                })
                .build(); */



        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(48, -38.12, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(32.4, -47.2, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3right= drive.trajectorySequenceBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(36, -10, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(-35, -11.8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3right.end())
                .splineToConstantHeading(new Vector2d(-26, -10.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-30.2, -28), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-51.64, -24.968789), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.18,()->{      //50.2     //-25.296789
                    teleop.lift.preload();
                })
                .addTemporalMarker(3.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -7.7), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -9.8), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.39, -13.158),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //62.49  //-11.658

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(30, -9.56),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -9.56),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.7,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-50, -32.3), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.852,()->{    //-49.8    -30
                    teleop.lift.preloadServo();
                    teleop.lift.rotation.setPosition(0.477);
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -6.1), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })
                .addTemporalMarker(1.4,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.4);
                    // while(teleop.intake.intakeUAD.getPosition()==0.4) {
                    //   collect();
                    //}

                })
                .addTemporalMarker(1.9,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);
                    stopintake();

                })


//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(17, -6.3), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                                      20      -6
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-61.67, -14.33),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.65791, -14.4512),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //62.92791       //-14.5912


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(30, -10.1),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -10.3),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-50.2, -37.9), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.52,()->{       //-50   -34.5
                    teleop.lift.preloadServo();
                })              //2.5
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(-40, -32),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    //  teleop.lift.Retractfinal();
                    targetPosition=2000;
                })
                .build();




        while (opModeInInit()) {
            telemetry.addData("case",processor.detection);
            telemetry.addData("right",processor.rightSum);
            telemetry.addData("middle",processor.middleSum);
            dashboard.setTelemetryTransmissionInterval(55);
            telemetry.update();

        }







        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            if(processor.detection==1){
                left();
            }

            if(processor.detection==2){
                center();
            }

            if(processor.detection==3){
                right();
            }
        }



    }


  /*
  public void center(){

      pos1 = drive.trajectoryBuilder(startPose)
              .lineToLinearHeading(new Pose2d(-37, -35, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      pos2 = drive.trajectoryBuilder(pos1.end())
              .lineToLinearHeading(new Pose2d(-50, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      pos3 = drive.trajectoryBuilder(pos2.end())
              .lineToLinearHeading(new Pose2d(-48, -17, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();

      pos4 = drive.trajectoryBuilder(pos3.end())
              .lineToLinearHeading(new Pose2d(29, -17, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .build();
      pos5 = drive.trajectoryBuilder(pos4.end())
              .splineToConstantHeading(new Vector2d(51.4, -41), Math.toRadians(0))
              .addTemporalMarker(0.001,()->{
                  teleop.lift.preload();
              })
              .addTemporalMarker(2.2,()->{
                  teleop.lift.preloadServo();
              })
              .build();

      pos6 = drive.trajectoryBuilder(pos5.end())
              .lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addTemporalMarker(0.001,()->{
                  teleop.lift.RetractServo();
              })
              .addTemporalMarker(1.5,()->{
                  teleop.lift.Retract();
              })
              .build();

      pos7 = drive.trajectoryBuilder(pos6.end())
              .lineToLinearHeading(new Pose2d(-61.1, -15.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
       .build();
posback = drive.trajectoryBuilder(pos7.end())
              .forward(4)
              .build();
      pos8 = drive.trajectoryBuilder(posback.end())
              .lineToLinearHeading(new Pose2d(41, -12, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addTemporalMarker(0.2,()->{
                  exit();

              })

              .addTemporalMarker(3.3,()->{
                  teleop.lift.cycle();
              })
              .build();



      pos9 = drive.trajectoryBuilder(pos8.end())
              .lineToLinearHeading(new Pose2d(49, -38.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addTemporalMarker(1,()->{
                  teleop.lift.preloadServo();
              })
              .build();

      pos10 = drive.trajectoryBuilder(pos9.end())
              .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addTemporalMarker(0.1,()->{
                  teleop.lift.RetractServo();
              })
              .addTemporalMarker(1.5,()->{
                  teleop.lift.Retract();
              })
              .build();




      drive.followTrajectory(pos1);
      drive.followTrajectory(pos2);
      drive.followTrajectory(pos3);
      drive.followTrajectory(pos4);
      sleep(200);
      drive.followTrajectory(pos5);
      sleep(200);
      teleop.lift.servoPixel.setPower(1);
      sleep(700);
      teleop.intake.intakeUAD.setPosition(0.3);
      drive.followTrajectory(pos6);
      drive.followTrajectory(pos7);
      collect();
      sleep(500);
      stopintake();
      sleep(300);
      teleop.intake.intakeUAD.setPosition(0.32);
      collect();
      sleep(900);
      teleop.intake.intakeUAD.setPosition(0.45);
      collect();
      drive.followTrajectory(pos8);
      exit();
      sleep(100);
      collect();
      sleep(400);
      drive.followTrajectory(pos9);
      teleop.lift.servoPixel.setPower(1);
      sleep(700);
      teleop.lift.RetractServo();
      sleep(200);
      teleop.lift.Retract();
      drive.followTrajectory(pos10);
      sleep(8000);







  } */

    public void center(){

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(32, -34.4, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                         // -37    -34.4

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(50, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(48, -7, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3.end())
                .splineToConstantHeading(new Vector2d(-26, -8.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-30.2, -28), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-50.64, -29.65), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.18,()->{      //50.24     //-29.65
                    teleop.lift.preload();
                })
                .addTemporalMarker(3.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(-37, -5.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -4.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{            //-7.5
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -4.5)/*-9.5*/, Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.416, -13.21),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //63  //-11.4

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(30, -9.56),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -9.56),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.7,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-50, -31), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.852,()->{
                    teleop.lift.preloadServo();
                    teleop.lift.rotation.setPosition(0.477);
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(-37, -5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -4.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{     //-25     -6
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })
                .addTemporalMarker(1.4,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.4);
                    // while(teleop.intake.intakeUAD.getPosition()==0.4) {
                    //   collect();
                    //}

                })
                .addTemporalMarker(1.9,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);
                    stopintake();

                })


//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(17, -6), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                                      17      -6
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-61.67, -14.33),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.80999, -13.778),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //63.223         //-13.708


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(30, -7),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -7.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.75,()->{  //1.8
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.9, -29), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.87,()->{ //2.35 //2.5  -49.7   -30
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(-40, -32),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    // teleop.lift.Retractfinal();
                    targetPosition=2000;
                })
                .build();


        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(315);  //320
        teleop.intake.intakeUAD.setPosition(0.178);//0.2565 //25 //0.257
        drive.followTrajectory(pos6);
        collect();
        sleep(80);
        teleop.intake.intakeUAD.setPosition(0.184);//0.2675 //252  //0.266  //0.268  //0.275
        sleep(197);
        teleop.intake.intakeUAD.setPosition(0.195);
//        collect();
        drive.followTrajectory(pos8);
        teleop.lift.servoPixel.setPower(1);
        sleep(446); //460
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.24); //0.346
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.258); //0.38
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.33);
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(446); //500
        drive.followTrajectory(pos12);
        sleep(2000);





     /*   pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(32, -34.4, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                         // -37    -34.4

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(50, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(48, -17, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3.end())
                .splineToConstantHeading(new Vector2d(-26, -18.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-50.2, -39), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -15), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, -16.79), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(63.387, -14.4),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(30, -14.8),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -14.8),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.65,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.8, -34), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -16), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })
                .addTemporalMarker(1.4,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.4);
                    while(teleop.intake.intakeUAD.getPosition()==0.4) {
                        collect();
                    }

                })
                .addTemporalMarker(2.5,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(20, -16.8), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(61.67, -14.43),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(30, -15.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -15.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.7, -39.5), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(-40, -36),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    teleop.lift.Retract();
                })
                .build();





        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.257);
        drive.followTrajectory(pos6);
        collect();
        sleep(80);
        teleop.intake.intakeUAD.setPosition(0.268);    //0.275
        sleep(200);
//        teleop.intake.intakeUAD.setPosition(0.45);
//        collect();
        drive.followTrajectory(pos8);
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.346);
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.38);
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.42);
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos12);
        sleep(2000); */






    }

    public void right(){   // left e de fapt

     /*   pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-31.7, -35.78, Math.toRadians(38)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-46, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-44, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(29.8, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(51.6, -49), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.2,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, -14.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-60.7, -17.3, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(37.8, -14.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(48.44, -39.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3 ,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();




        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        sleep(200);
        drive.followTrajectory(pos5);
        sleep(200);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.intake.intakeUAD.setPosition(0.3);
        drive.followTrajectory(pos6);
        drive.followTrajectory(pos7);
        collect();
        sleep(500);
        stopintake();
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.32);
        collect();
        sleep(900);
        teleop.intake.intakeUAD.setPosition(0.45);
        collect();
        drive.followTrajectory(pos8);
        exit();
        sleep(100);
        collect();
        sleep(400);
        drive.followTrajectory(pos9);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.lift.RetractServo();
        sleep(200);
        teleop.lift.Retract();
        drive.followTrajectory(pos10);
        sleep(8000); */

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(31.7, -35.78, Math.toRadians(142)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(46, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(44, -10, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3.end())
                .splineToConstantHeading(new Vector2d(-26, -11.2), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-30.2, -28), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-51, -34.235), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{  //1.2     //50.55 //-34.435
                    teleop.lift.preload();
                })
                .addTemporalMarker(3.25,()->{   //3.25
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-10, -2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.475, -12.9),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //63.07  //-11.09
                                 //63.07  //-11.09

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(30, -9.16),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25.3, -9.16),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{   //   -25     //-9.56
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.7,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-49.94, -24.5), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.9,()->{               //-25
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(-25, -4.3), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{            //-5.3
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);

                })
                .addTemporalMarker(1.4,()->{
                    teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.4);
                    // while(teleop.intake.intakeUAD.getPosition()==0.4) {
                    //   collect();
                    //}

                })
                .addTemporalMarker(1.9,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.22);  //0.3
                    stopintake();

                })


//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(17, -5.32), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                                      20      -5.29
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-61.67, -14.33),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(62.772, -13.57),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                              //63.3        //-12.54


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(30, -10),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-25, -10.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(-50.2, -28.95), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{      //-50     -29.3
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(-40, -32),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    // teleop.lift.Retractfinal();
                    targetPosition=2000;
                })
                .build();


        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.180); //0.256 //0.257
        drive.followTrajectory(pos6);
        collect();
        sleep(90);
        teleop.intake.intakeUAD.setPosition(0.183); //0.26 //0.266  //0.268  //0.275
        sleep(210);
        teleop.intake.intakeUAD.setPosition(0.20); // asta era comentata
//        collect();
        drive.followTrajectory(pos8);
        exit();
        teleop.lift.servoPixel.setPower(1);
        sleep(457); //500
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.244); //0.296 //0.346
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.2576); // 0.24 //0.38
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.3207);// 0.28  //0.42
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(457); //500
        drive.followTrajectory(pos12);
        sleep(2000);




    }


    public void left(){  //right e de fapt


        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectorySequence(pos3right);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.18);// 0.256//257 //25 //0.257
        drive.followTrajectory(pos6);
        collect();
        sleep(80);
        teleop.intake.intakeUAD.setPosition(0.183);//0.263//26589 // 268 //252  //0.266  //0.268  //0.275
        sleep(200);
        teleop.intake.intakeUAD.setPosition(0.20);
//        collect();
        drive.followTrajectory(pos8);
        exit();
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.245);  //0.3445 //346
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.257); //0.38
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.3207);  //0.42
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos12);
        sleep(2000);






    }




    public void collect (){
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(-1);
        teleop.lift.servoPixel.setPower(-1);
        teleop.lift.rotation.setPosition(0.477);

        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(-1);
        teleop.lift.servoPixel.setPower(-1);
    }
    public void exit (){
        teleop.intake.intakeMotor.setPower(1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(0);
        teleop.intake.intakeUAD.setPosition(0.2);

        teleop.intake.intakeMotor.setPower(1);
        teleop.intake.intakeMotorRight.setPower(1);
    }

    public void stopintake (){
        teleop.intake.intakeMotor.setPower(0);
        teleop.intake.intakeMotorRight.setPower(0);
}







}
