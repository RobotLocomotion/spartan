'''
Usage:

directorPython scripts/testOnlinePipeline.py --logFolder logs/moving-camera --bot-config $SPARTAN_SOURCE_DIR/apps/iiwa/iiwaManip.cfg
'''

from director import drcargs
import corl.onlinepipeline

if __name__ == '__main__':

    parser = drcargs.getGlobalArgParser().getParser()
    parser.add_argument('--logFolder', type=str, dest='logFolder', help='location of top level folder for this log, relative to CorlDev/data')
    args = parser.parse_args()
    print "logFolder = ", args.logFolder
    corl.onlinepipeline.processImages(args.logFolder)

