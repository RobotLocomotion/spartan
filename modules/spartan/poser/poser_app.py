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

# spartan
from spartan.poser.poser_visualizer import PoserVisualizer

"""
Launches a poser client app
"""

VISUALIZE = True
POSER_OUTPUT_FOLDER = os.getenv("POSER_SANDBOX_DIR")


if __name__ == "__main__":


    globalsDict = globals()

    if VISUALIZE:
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

    poser_vis = PoserVisualizer(POSER_OUTPUT_FOLDER)


    def poser_vis_function():
        poser_response = poser_vis.load_poser_response()
        poser_vis.visualize_result(poser_response)



    TimerCallback(callback=poser_vis_function).singleShot(0)
    app.app.start(restoreWindow=True)


