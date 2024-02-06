package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.CommandScheduler.telem
import com.amarcolini.joos.control.DCMotorFeedforward
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.followers.HolonomicGVFFollower
import com.amarcolini.joos.followers.HolonomicPIDVAFollower
import com.amarcolini.joos.followers.PathFollower
import com.amarcolini.joos.followers.TrajectoryFollower
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.DrivePathFollower
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower
import com.amarcolini.joos.hardware.drive.MecanumDrive
import com.amarcolini.joos.hardware.drive.Standard3WheelLocalizer
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg
import org.firstinspires.ftc.teamcode.opmode.GVFTest

@JoosConfig
class Drivetrain(
    motors: MotorGroup,
    val encoders: List<Motor.Encoder>
) : MecanumDrive(
    motors,
    Companion.trackWidth, Companion.trackWidth, Companion.lateralMultiplier
), DriveTrajectoryFollower, DrivePathFollower {
    val parallelEncoders = listOf(encoders[0], encoders[1])
    val perpEncoder = encoders[2]

    override val constraints = MecanumConstraints(
        48.0,
        trackWidth,
        wheelBase,
        lateralMultiplier,
        48.0,
        45.0,
        280.deg,
        250.deg
    )

    val motorGroup = motors

    override val trajectoryFollower: TrajectoryFollower = HolonomicPIDVAFollower(
        axialCoeffs, lateralCoeffs, headingCoeffs, Pose2d(0.5, 0.5, 2.deg), 0.5
    )

    private val poseHistory = ArrayList<Pose2d>()
    var pathColor: String = "#4CAF50"
    var turnColor: String = "#7c4dff"
    var waitColor: String = "#dd2c00"
    var robotColor: String = "#3F51B5"

    /**
     * Whether current drive pose, trajectory, and pose history are to be displayed
     * on FTC Dashboard.
     */
    var dashboardEnabled: Boolean = true
    var poseHistoryLimit: Int = 100

    override var pathFollower: PathFollower = HolonomicGVFFollower(
        constraints.maxVel,
        constraints.maxAccel,
        constraints.maxAccel,
        constraints.maxAngVel,
        constraints.maxAngAccel,
        Pose2d(0.5, 0.5, 5.deg),
        GVFTest.kN, GVFTest.kOmega, GVFTest.kX, GVFTest.kY, GVFTest.pidCoefficients,
    )

    override fun update() {
        super<MecanumDrive>.update()
        if (dashboardEnabled) {
            val trajectory = getCurrentTrajectory()
            if (trajectory != null) {
                telem.drawSampledTrajectory(trajectory, pathColor, turnColor, waitColor)
                poseHistory.add(poseEstimate)
                if (poseHistoryLimit > -1 && poseHistory.size > poseHistoryLimit)
                    poseHistory.removeFirst()
                telem.drawPoseHistory(poseHistory, robotColor)
                telem.drawRobot(trajectory[trajectoryFollower.elapsedTime()], pathColor)
            }
            telem.fieldOverlay().setStrokeWidth(1)
            telem.drawRobot(poseEstimate, robotColor)
        }
    }

    fun pathCommandBuilder(
        startPose: Pose2d = poseEstimate,
        startTangent: Angle = startPose.heading
    ) = PathCommandBuilder(this, startPose, startTangent)

    companion object {
        val feedforwardCoeffs = DCMotorFeedforward(0.017, 0.0002, 0.05)
        var distPerTick = 0.002956143499653
        var leftPose = Pose2d(0.0, 7.325, 0.deg)
        var rightPose = Pose2d(0.0, -7.325, 0.deg)
        var perpPose = Pose2d(5.801, 0.0, (-90).deg)
        var trackWidth = 13.5
        var lateralMultiplier = 0.7

        val axialCoeffs = PIDCoefficients(8.0, 0.0, 0.2)
        val lateralCoeffs = PIDCoefficients(8.0, 0.0, 0.2)
        val headingCoeffs = PIDCoefficients(6.0, 0.0, 0.2)
    }

    init {
        motors.apply {
            zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
            feedforward = feedforwardCoeffs
            runMode = Motor.RunMode.RUN_WITHOUT_ENCODER
            distancePerTick = 1.0
        }

        encoders.forEach {
            it.distancePerTick = distPerTick
        }

        localizer = Standard3WheelLocalizer(
            encoders, listOf(leftPose, rightPose, perpPose)
        )
    }
}