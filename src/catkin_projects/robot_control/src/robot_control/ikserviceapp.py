#!/usr/bin/env python


from director import mainwindowapp
from director import drcargs
from director import robotsystem
from director import applogic
from director import consoleapp


from robot_control.ikservice import IkService

if __name__ == '__main__':

    # parse args first
    parser = drcargs.getGlobalArgParser().getParser()
    # parser.add_argument('--logFolder', type=str, dest='logFolder',
    #                       help='location of top level folder for this log, relative to LabelFusion/data')

    args = parser.parse_args()
    # print 'log folder:', args.logFolder

    # construct the app
    fields = mainwindowapp.construct()
    factory = robotsystem.ComponentFactory()
    factory.register(robotsystem.RobotSystemFactory)
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True, useSegmentation=True,
                                useSegmentationAffordances=True)
    robotSystem = factory.construct(view=fields.view, options=options)

    # use pydrake ik backend
    ikPlanner = robotSystem.ikPlanner
    ikPlanner.planningMode = 'pydrake'
    ikPlanner.plannerPub._setupLocalServer()


    ikService = IkService(robotSystem)

    myObjects = dict()
    myObjects['ikService'] = ikService
    ikService.run()

    # these lines are used to update the globals for the interactive python console
    fields.globalsDict.update(**dict(fields))
    globals().update(**fields.globalsDict)
    globals().update(**myObjects)

    # toggle show/hide of window
    showWindow = False
    if showWindow:
        fields.app.start()
    else:
        consoleapp.ConsoleApp.start()



