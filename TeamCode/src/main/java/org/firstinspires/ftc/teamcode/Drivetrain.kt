package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandScheduler.telem
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.FunctionalCommand
import com.amarcolini.joos.control.DCMotorFeedforward
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.drive.AbstractMecanumDrive
import com.amarcolini.joos.drive.DriveSignal
import com.amarcolini.joos.followers.HolonomicGVFFollower
import com.amarcolini.joos.followers.HolonomicPIDVAFollower
import com.amarcolini.joos.followers.PathFollower
import com.amarcolini.joos.followers.TrajectoryFollower
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.MotorGroup
import com.amarcolini.joos.hardware.drive.DrivePathFollower
import com.amarcolini.joos.localization.ThreeTrackingWheelLocalizer
import com.amarcolini.joos.trajectory.Trajectory
import com.amarcolini.joos.trajectory.TrajectoryBuilder
import com.amarcolini.joos.trajectory.constraints.MecanumConstraints
import com.amarcolini.joos.util.deg
import org.firstinspires.ftc.teamcode.opmode.auto.GVFTest

@JoosConfig
class Drivetrain(
    val motors: MotorGroup,
    val encoders: List<Motor.Encoder>
) : AbstractMecanumDrive(
    Companion.trackWidth, Companion.trackWidth, Companion.lateralMultiplier
), Component, DrivePathFollower {
    val parallelEncoders = listOf(encoders[0], encoders[1])
    val perpEncoder = encoders[2]

    val constraints = MecanumConstraints(
        48.0,
        trackWidth,
        wheelBase,
        lateralMultiplier,
        48.0,
        38.0,
        280.deg,
        250.deg
    )

    val trajectoryFollower: TrajectoryFollower = HolonomicPIDVAFollower(
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

    override fun setRunMode(runMode: Motor.RunMode) {
        motors.runMode = runMode
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: Motor.ZeroPowerBehavior) {
        motors.zeroPowerBehavior = zeroPowerBehavior
    }

    override fun update() {
        updatePoseEstimate()
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

    /**
     * Returns a [TrajectoryBuilder] with the constraints of this drive.
     */
    @JvmOverloads
    fun trajectoryBuilder(
        startPose: Pose2d = poseEstimate,
        startTangent: Angle = startPose.heading
    ): TrajectoryBuilder = TrajectoryBuilder(
        startPose,
        startTangent,
        constraints.velConstraint,
        constraints.accelConstraint,
        constraints.maxAngVel, constraints.maxAngAccel, constraints.maxAngJerk
    )


    /**
     * Returns a [TrajectoryBuilder] with the constraints of this drive.
     */
    @JvmOverloads
    fun trajectoryBuilder(
        startPose: Pose2d = poseEstimate,
        reversed: Boolean
    ): TrajectoryBuilder = TrajectoryBuilder(
        startPose,
        reversed,
        constraints.velConstraint,
        constraints.accelConstraint,
        constraints.maxAngVel, constraints.maxAngAccel, constraints.maxAngJerk
    )

    /**
     * Returns a [Command] that follows the provided trajectory.
     */
    fun followTrajectory(trajectory: Trajectory): Command {
        poseHistory.clear()
        return if (!trajectoryFollower.isFollowing()) {
            FunctionalCommand(
                init = { trajectoryFollower.followTrajectory(trajectory) },
                execute = {
                    setDriveSignal(trajectoryFollower.update(poseEstimate, poseVelocity).also {
                        telem.addData("driveSignal", it)
                    })
                },
                isFinished = { !trajectoryFollower.isFollowing() },
                end = { setDriveSignal(DriveSignal()) },
                requirements = setOf(this)
            )
        } else Command.emptyCommand()
    }

    /**
     * Returns the trajectory currently being followed by this drive, if any.
     */
    fun getCurrentTrajectory(): Trajectory? =
        if (trajectoryFollower.isFollowing()) trajectoryFollower.trajectory else null

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

        localizer = Odometry()
    }

    private inner class Odometry : ThreeTrackingWheelLocalizer(
        listOf(
            leftPose,
            rightPose,
            perpPose
        )
    ) {
        override fun getWheelPositions(): List<Double> = encoders.map { it.distance }

        override fun getWheelVelocities(): List<Double> = encoders.map { it.distanceVelocity }
    }

    override fun getWheelPositions(): List<Double> = motors.getDistances()
    override fun getWheelVelocities(): List<Double> = motors.getDistanceVelocities()

    override fun setMotorPowers(
        frontLeft: Double,
        backLeft: Double,
        backRight: Double,
        frontRight: Double
    ) = motors.setPowers(listOf(frontLeft, backLeft, backRight, frontRight))

    override fun setWheelVelocities(velocities: List<Double>, accelerations: List<Double>) {
        motors.setDistanceVelocities(velocities, accelerations)
    }
}