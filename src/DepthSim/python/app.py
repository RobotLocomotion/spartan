from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import consoleapp
from director import transformUtils
from director import mainwindowapp


class CameraPoses(object):

    def __init__(self, posegraphFile=None):
        self.posegraphFile = posegraphFile

        if self.posegraphFile is not None:
            self.loadCameraPoses(posegraphFile)


    def loadCameraPoses(self, posegraphFile):
        data = np.loadtxt(posegraphFile)
        self.poseTimes = np.array(data[:,0]*1e6, dtype=int)
        self.poses = []
        for pose in data[:,1:]:
            pos = pose[:3]
            quat = pose[6], pose[3], pose[4], pose[5] # quat data from file is ordered as x, y, z, w
            self.poses.append((pos, quat))

    def getCameraPoseAtUTime(self, utime):
        idx = np.searchsorted(self.poseTimes, utime, side='left')
        if idx == len(self.poseTimes):
            idx = len(self.poseTimes) - 1

        (pos, quat) = self.poses[idx]
        return transformUtils.transformFromPose(pos, quat)

def setCameraTransform(camera, transform):
    '''Set camera transform so that view direction is +Z and view up is -Y'''
    origin = np.array(transform.GetPosition())
    axes = transformUtils.getAxesFromTransform(transform)
    camera.SetPosition(origin)
    camera.SetFocalPoint(origin+axes[2])
    camera.SetViewUp(-axes[1])

def setCameraInstrinsicsAsus(view):
    principalX = 320.0
    principalY = 240.0
    focalLength = 528.0
    setCameraIntrinsics(view, principalX, principalY, focalLength)

def setCameraIntrinsics(view, principalX, principalY, focalLength):
    '''Note, call this function after setting the view dimensions'''

    imageWidth = view.width
    imageHeight = view.height

    wcx = -2*(principalX - float(imageWidth)/2) / imageWidth
    wcy =  2*(principalY - float(imageHeight)/2) / imageHeight
    viewAngle = focalLengthToViewAngle(focalLength, imageHeight)

    camera = view.camera()
    camera.SetWindowCenter(wcx, wcy)
    camera.SetViewAngle(viewAngle)



if __name__ == '__main__':
	app = mainwindowapp.construct()#globals?
	applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)
	view = app.view

	data_dir = sys.argv[1]
	num_im = int(sys.argv[1])
	mesh = sys.argv[3]
	#load mesh
	polyData = ioUtils.readPolyData(data_dir+'/'+sys.argv[3])
	obj = vis.showPolyData(polyData, name=mesh, parent=folder, color=color)

	view.setFixedSize(640, 480)
	setCameraInstrinsicsAsus(view)
    # cameraToWorld = utils.getDefaultCameraToWorld()
    # setCameraTransform(view.camera(), cameraToWorld)
	setCameraTransform(view.camera(), vtk.vtkTransform())
	view.forceRender()
	print "rendered"
    #objToWorld = transformUtils.transformFromPose(*data['pose'])
    #self.objectToWorld[objName] = objToWorld
    #obj.actor.SetUserTransform(objToWorld)

    # applogic.setCameraTerrainModeEnabled(self.sceneView, True)

#put object in correct place???
	poses = CameraPoses(data_dir+"/posegraph.posegraph")
	for i in num_im:
	      utimeFile = open(data_dir+"/images/"+ im_num + "_utime.txt", 'r')
	      utime = int(utimeFile.read())
	      print utime
	      # update camera transform
	      cameraToCameraStart =poses.getCameraPoseAtUTime(utime)
	      t = cameraToCameraStart
	      vis.updateFrame(t, 'camera pose')
	      setCameraTransform(self.view.camera(), t)

	      cameraPose = om.findObjectByName('camera pose')
	      cameraPose.setProperty('Visible', False)


        #now get normal map and depth map