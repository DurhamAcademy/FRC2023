package frc.kyberlib.lighting.animations

import frc.kyberlib.math.units.extensions.Time
import frc.kyberlib.math.units.extensions.seconds
import java.awt.Color

class AnimationRGBWave(private val cycles: Double = 1.0, val secondsPerMovement: Time = .2.seconds, val reversed: Boolean = false, enableTransparency: Boolean = false, condition: ()->Boolean = { true }) : LEDAnimation(condition, enableTransparency) {

    private var buf: MutableList<Color> = ArrayList<Color>()
    private var t = 0
    fun constructInitialBuffer(length: Int): MutableList<Color> {

        return if (cycles >= 1) {
            Array<Color>(length) {
                val hue = (it % (length / cycles)) / (length / cycles)
                Color.getHSBColor(hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        } else {
            Array<Color>((length / cycles).toInt()) {
                val hue = it / (length / cycles)
                Color.getHSBColor(hue.toFloat(), 1F, 1F)
            }.asList().toMutableList()
        }
    }

    override fun getBuffer(time: Time, length: Int): List<Color> {
        if(buf.size != length) {
            buf = constructInitialBuffer(length)
        }

        val curT = (time / secondsPerMovement).toInt()
        if(curT != t) {
            t = curT
            buf.add(0, buf.removeAt(buf.size - 1))
        }
//        if (reversed) buf.reverse()

        return buf.take(length)
    }
}
