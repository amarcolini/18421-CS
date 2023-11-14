package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class DroneLauncher(private val servo: Servo) : AbstractComponent() {

    companion object {
        var closePosition = 0.05
        var openPosition = 0.4
    }

    init {
        subcomponents += servo
    }

    fun reset() {
        servo.position = closePosition
    }

    fun launch() {
        servo.position = openPosition
    }
}