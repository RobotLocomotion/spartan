from director.lcmlogplayer import LcmLogPlayer, LcmLogPlayerGui
from director import lcmUtils
from director import visualization as vis
import bot_core as lcmbotcore


def getDrakeSimTimeForEvent(event):
    if event.channel == 'DRAKE_VIEWER_DRAW':
        msg = lcmbotcore.viewer_draw_t.decode(event.data)
        return msg.timestamp*1000


def initDrakeTimeDisplay():
    def onViewerDraw(msg):
        t = msg.timestamp*1e-3
        vis.updateText('sim time: %.3f' % t, 'sim time')
    lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmbotcore.viewer_draw_t, onViewerDraw)


if __name__ == '__main__':

    if len(_argv) != 2:
        raise Exception('usage: %s <filename.lcmlog>' % _argv[0])

    filename = _argv[1]

    logPlayer = LcmLogPlayer()
    logPlayer.playbackFactor = 1.0
    logPlayer.readLog(filename, eventTimeFunction=getDrakeSimTimeForEvent)
    logPlayer.skipToTime(0.0)

    logPlayerGui = LcmLogPlayerGui(logPlayer)
    logPlayerGui.playbackStartTime = 0.01

    initDrakeTimeDisplay()
