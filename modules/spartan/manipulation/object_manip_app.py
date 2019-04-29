#!/usr/bin/env directorPython

# system
import os
import argparse

# director
import director.vtkAll as vtk
import director.vtkNumpy as vnp
import director.objectmodel as om
import director.visualization as vis
from director import mainwindowapp
from director.timercallback import TimerCallback
from director import ioUtils

import spartan.utils.utils as spartanUtils
from spartan.manipulation.object_manipulation import ObjectManipulation
from spartan.poser.poser_visualizer import PoserVisualizer



"""
Launches a poser client app
"""




if __name__ == "__main__":


    globalsDict = globals()

    app = mainwindowapp.construct()
    app.gridObj.setProperty('Visible', True)
    app.viewOptions.setProperty('Orientation widget', True)
    app.viewOptions.setProperty('View angle', 30)
    app.sceneBrowserDock.setVisible(True)
    app.propertiesDock.setVisible(False)
    app.mainWindow.setWindowTitle('Depth Scanner')
    app.mainWindow.show()
    app.mainWindow.resize(920, 600)
    app.mainWindow.move(0, 0)

    view = app.view

    globalsDict['app'] = app
    globalsDict['view'] = view

    # load background scene if it exists
    background_ply_file = os.path.join(spartanUtils.get_data_dir(), 'pdc', 'logs_special',
                                       '2019-01-03-22-43-55', 'processed', 'fusion_mesh.ply')


    def visualize_background():
        if not os.path.exists(background_ply_file):
            return

        parent = om.getOrCreateContainer("scene")
        poly_data = ioUtils.readPolyData(background_ply_file)
        vis.updatePolyData(poly_data, 'table', parent=parent)


    visualize_background()

    poser_vis = PoserVisualizer.make_default()
    object_manip = ObjectManipulation(poser_visualizer=poser_vis)
    globalsDict['object_manip'] = object_manip
    globalsDict['o'] = object_manip


    def single_shot_function():
        object_manip.load_and_visualize_mug_model()


    TimerCallback(callback=single_shot_function).singleShot(0)
    app.app.start(restoreWindow=True)


