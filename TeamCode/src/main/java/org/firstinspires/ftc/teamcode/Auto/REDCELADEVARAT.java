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


@Autonomous(name ="REDCELADEVARAT", group = "CENTERSTAGE")

public class REDCELADEVARAT extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-39.2, -64, Math.toRadians(90));

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
    PropDetectionRedFar processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        teleop.airplane.initAirplane(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        processor = new PropDetectionRedFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();


        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-45, -38.12, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-32.4, -47.2, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3right= drive.trajectorySequenceBuilder(pos2.end())  //11.2
                .lineToLinearHeading(new Pose2d(-31.8, -11.39, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(-35, -11.8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        //to pannel
        pos4 = drive.trajectoryBuilder(pos3right.end())
                .splineToConstantHeading(new Vector2d(26, -16.2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(51, -32.61), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{     // 50.7/ 50.94    -32.51
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(25, -15), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -17.14), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.4299, -14.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                              //-62.22

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(-30, -14.78),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -14.78),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{            //-14.9 la ambele
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.65,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.8, -39), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(25, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                    // collect();
                    //}

                })
                .addTemporalMarker(1.9,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.25); //0.3
                    stopintake();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -15.26),/*-15.3*/ Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.57, -13.65),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                            //-61.67


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(-30, -13.28),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -13.28),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{             //-13.4 la ambele
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.7, -39.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(40, -36),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    //teleop.lift.Retract();
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
                .lineToLinearHeading(new Pose2d(-28.6, -34.4, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                         // -32    -34.4

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-50, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-48, -17, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3.end())      //25  -18.2
                .splineToConstantHeading(new Vector2d(25, -18.2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(30, -28.2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(50.7, -37.85), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{    //50.2    -39
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.3,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(25, -15), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -16.79), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.4412, -14.2),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                     //       -62.7512        -13.2

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(-30, -14.8),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -14.8),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.65,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.8, -33.6), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{  //   49.8     -34
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(25, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .addTemporalMarker(1.9,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{   //2.5
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);
                    stopintake();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -15/*-16.8*/), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.551, -13.57),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                             //-61.65      -13.07


        pos11 = drive.trajectoryBuilder(pos10.end())
                .splineToConstantHeading(new Vector2d(-30, -14.5),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -14.5),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{            //-15.2
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.7, -39.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(40, -36),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    //  teleop.lift.Retract();
                    targetPosition=2000;
                })
                .build();





      /*  drive.followTrajectory(pos1);
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

      /*  drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        sleep(200);
       // drive.followTrajectory(pos5);
        sleep(200);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.intake.intakeUAD.setPosition(0.2);
        drive.followTrajectory(pos6);
       // drive.followTrajectory(pos7);
        collect();
        sleep(500);
        stopintake();
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.25);
        collect();
        sleep(900);
       // teleop.intake.intakeUAD.setPosition(0.45);
        collect();
        drive.followTrajectory(pos8);
        exit();
        sleep(100);
        collect();
        sleep(400);
       // drive.followTrajectory(pos9);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.lift.RetractServo();
        sleep(200);
        teleop.lift.Retract();
        drive.followTrajectory(pos10);
        sleep(8000); */

        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(330);
        teleop.intake.intakeUAD.setPosition(0.18);
        drive.followTrajectory(pos6);
        collect();
        sleep(90);
        teleop.intake.intakeUAD.setPosition(0.184);    //0.275
        sleep(230);
        teleop.intake.intakeUAD.setPosition(0.2);
//        collect();
        drive.followTrajectory(pos8);
        teleop.lift.servoPixel.setPower(1);
        sleep(450); //500
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.24);
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.253);
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.327);
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(450); //500
        drive.followTrajectory(pos12);
        sleep(2000);






    }

    public void right(){

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
                .lineToLinearHeading(new Pose2d(-31.7, -35.78, Math.toRadians(38)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-46, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-44, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //to pannel
        pos4 = drive.trajectoryBuilder(pos3.end())
                .splineToConstantHeading(new Vector2d(26, -18.2), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(30.2, -30), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(50.9, -44), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.2,()->{      //50.4
                    teleop.lift.preload();
                })
                .addTemporalMarker(3.25,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        //to stack
        pos6 = drive.trajectoryBuilder(pos4.end(),true)
                .splineToConstantHeading(new Vector2d(25, -15), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.8,()->{
                    teleop.lift.Retract();
                })

//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.4, -13.938),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                               //-63.09   -12.81

//to pannel 2
//        posback = drive.trajectoryBuilder(pos6.end())
//                .forward(4)
//                .build();
//
//
        pos8 = drive.trajectoryBuilder(pos6.end())
                .splineToConstantHeading(new Vector2d(-30, -14.56),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -14.56),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    collect();
                })
                .addTemporalMarker(0.3,()->{
                    collect();
                })
                .addTemporalMarker(0.65,()->{
                    exit();
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.74, -34.5), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{      //49.74     -36
                    teleop.lift.preloadServo();
                })
                .build();


//to stack 2
        pos10 = drive.trajectoryBuilder(pos8.end(),true)
                .splineToConstantHeading(new Vector2d(25, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .addTemporalMarker(1.8,()->{
                    collect();

                })
                .addTemporalMarker(4.3,()->{
                    // teleop.lift.Retract();
                    teleop.intake.intakeUAD.setPosition(0.3);
                    stopintake();

                })


//                .splineToConstantHeading(new Vector2d(10, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-20, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-40, -16), Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.splineToConstantHeading(new Vector2d(-61.67, -14.33),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-62.6, -13.4),Math.toRadians(180),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();                            //-63.372    -12.9


        pos11 = drive.trajectoryBuilder(pos10.end())          //-13.5
                .splineToConstantHeading(new Vector2d(-30, -13),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(25, -13.2),Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.65,()->{           //-13.7
                    exit();
                })
                .addTemporalMarker(0.63,()->{
                    teleop.intake.intakeMotor.setPower(1);
                    teleop.intake.intakeMotorRight.setPower(1);
                })

                .addTemporalMarker(1.8,()->{
                    teleop.lift.cycleHigh();
                })
                .splineToConstantHeading(new Vector2d(49.45, -35.3), Math.toRadians(0),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2.5,()->{       //49.45    -37
                    teleop.lift.preloadServo();
                })
                .build();


        pos12 = drive.trajectoryBuilder(pos11.end())
                .lineToConstantHeading(new Vector2d(40, -36),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.05,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(0.5,()->{
                    // teleop.lift.Retract();
                    targetPosition=2000;
                })
                .build();


        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.179);
        drive.followTrajectory(pos6);
        collect();
        sleep(90);
        teleop.intake.intakeUAD.setPosition(0.184);    //0.275
        sleep(230);
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
        teleop.intake.intakeUAD.setPosition(0.245);
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.258);
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.329);
        sleep(100); //70
        drive.followTrajectory(pos11);
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos12);
        sleep(2000);




    }


    public void left(){


        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectorySequence(pos3right);
        drive.followTrajectory(pos4);
        teleop.lift.servoPixel.setPower(0.9);
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.179);
        drive.followTrajectory(pos6);
        collect();
        sleep(90);
        teleop.intake.intakeUAD.setPosition(0.184);    //0.275
        sleep(230); //200
        teleop.intake.intakeUAD.setPosition(0.20);
//        collect();
        drive.followTrajectory(pos8);
        teleop.lift.servoPixel.setPower(1);
        sleep(460); //500
        drive.followTrajectory(pos10);
        collect();
        sleep(80);
        // teleop.intake.intakeUAD.setPosition(0.3);
        teleop.intake.intakeUAD.setPosition(0.245);
        sleep(120); //100
        teleop.intake.intakeUAD.setPosition(0.258);
        sleep(60); //30
        teleop.intake.intakeUAD.setPosition(0.329);
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
    }
    public void exit (){
        teleop.intake.intakeMotor.setPower(1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(0);
        teleop.intake.intakeUAD.setPosition(0.2);
    }

    public void stopintake (){
        teleop.intake.intakeMotor.setPower(0);
        teleop.intake.intakeMotorRight.setPower(0);
 }

}
