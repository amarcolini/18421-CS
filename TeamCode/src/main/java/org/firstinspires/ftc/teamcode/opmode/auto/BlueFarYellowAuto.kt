package org.firstinspires.ftc.teamcode.opmode.auto

import android.location.Location
import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.trajectory.Trajectory
import com.amarcolini.joos.trajectory.TrajectoryBuilder
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
@JoosConfig
class BlueFarYellowAuto : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(-16.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(-20.0, 36.0, (-180).deg)
        var centerPlopPose = Pose2d(-16.0, 45.0, (-90).deg)
        var leftPlopPose = Pose2d(-10.0, 36.0, (0).deg)
        var rightPlacePose = Pose2d(53.0, 32.0, 5.deg)
        var centerPlacePose = Pose2d(52.0, 41.0, 0.deg)
        var leftPlacePose = Pose2d(52.0, 48.0, 0.deg)
        var regularExitPose = Pose2d(-16.0, 10.0, 0.deg)
        var centerExitPose = Pose2d(-26.0, 10.0, 0.deg)
        var crossPose = Pose2d(48.0, 10.0, 0.deg)
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
    }

    override fun preStart() {
        cancelAll()
        robot.outtake.ready()
        robot.drive.poseEstimate = startPose
        val location = pipeline.lastKnownLocation

        val (plopPose, placePose) = when (location) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }

        val purplePlopTrajectory =
            robot.drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(plopPose)
                .build()
        val purpleBackUpTrajectory =
            robot.drive.trajectoryBuilder(purplePlopTrajectory.end())
                .back(3.0)
                .build()

//        val yellowPlaceTrajectory =
//            robot.drive.trajectoryBuilder(purplePlopTrajectory.end())
//                .back(3.0)
//                .lineToSplineHeading(placePose)
//                .build()

        val exitTrajectory = robot.drive.trajectoryBuilder(purpleBackUpTrajectory.end())
            .run {
                if (location == PropPipeline.PropLocation.Center) {
                    lineToSplineHeading(centerExitPose)
                } else lineToSplineHeading(regularExitPose)
                lineToSplineHeading(crossPose)
                lineToSplineHeading(placePose)
                build()
            }

        val purplePlopCommand = robot.drive.followTrajectory(purplePlopTrajectory)
            .then(robot.pixelPlopper.plop())
            .then(robot.drive.followTrajectory(purpleBackUpTrajectory))

        val yellowPlaceCommand = robot.drive.followTrajectory(exitTrajectory)
            .then(robot.outtake::extend)
            .wait(2.0)
            .then(robot.outtake::releaseRight)
            .wait(2.0)
            .then(robot.outtake::reset)
            .wait(2.0)

        SequentialCommand(
            true,
            purplePlopCommand,
            yellowPlaceCommand
        ).thenStopOpMode().schedule()
    }
}