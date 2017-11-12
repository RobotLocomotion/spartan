from director import mainwindowapp
from director import robotsystem
from director import applogic as app
from director import consoleapp
from director import transformUtils
from director import mainwindowapp
from director import depthscanner
from director import ioUtils
from director import visualization as vis
import numpy as np
from director import objectmodel as om
from director import vtkAll as vtk

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

def focalLengthToViewAngle(focalLength, imageHeight):
    '''Returns a view angle in degrees that can be set on a vtkCamera'''
    return np.degrees(2.0 * np.arctan2(imageHeight/2.0, focalLength))

def encode_normal_rgb(view, height, width, pickType='cells', tolerance=0.05):
	#picktype is one of ('points', 'cells', 'render')
	image = np.zeros((height,width,3))
	for i in range(height):
		print i
		for j in range(width):
			pickPointFields = vis.pickPoint(
			[i,j],
			view,
			pickType=pickType,
			tolerance=tolerance)
			normal = np.array(pickPointFields.pickedNormal)
			image[i,j,:] = normal
    # add some rgb conversion step, maybe png writer does that???
	return image

if __name__ == '__main__':
	#setup
	view_height = 640
	view_width = 480
	data_dir = sys.argv[1]
	num_im = int(sys.argv[2])
	mesh = sys.argv[3]

	#launch app
	application = mainwindowapp.construct() #globals?
	view = app.getCurrentRenderView()

	#load mesh TODO: load raw point cloud and convert to mesh automatically
	polyData = ioUtils.readPolyData(data_dir+'/'+sys.argv[3])
	obj = vis.showPolyData(polyData, name=mesh)
	
	#setup
	view.setFixedSize(view_height, view_width)
	setCameraInstrinsicsAsus(view)
	setCameraTransform(view.camera(), vtk.vtkTransform())
	view.camera().SetClippingRange(0.4, 5)
	view.forceRender()
	scanner = depthscanner.DepthScanner(view)

	poses = CameraPoses(data_dir+"/posegraph.posegraph")
	# toggle show/hide of window
	
	for i in range(1,num_im+1):
		print "rendering image" + str(i)
	   	utimeFile = open(data_dir+"/images/"+ str(i).zfill(10) + "_utime.txt", 'r')
	   	utime = int(utimeFile.read())

	   	#update camera transform
	   	cameraToCameraStart = poses.getCameraPoseAtUTime(utime)
	   	t = cameraToCameraStart
	   	vis.updateFrame(t, 'camera pose')
	   	setCameraTransform(view.camera(), t)
	   	cameraPose = om.findObjectByName('camera pose')
	   	cameraPose.setProperty('Visible', False)
	   	view.forceRender()

		vtk_depth_image = scanner.getDepthImage() 
		print "here"

		#encode normals into file
		#vtk_normal_image = encode_normal_rgb(view,view_height,view_width)
		#print "rendered normal image at " + str(utime)

		#write out files
		writer = vtk.vtkBMPWriter()
		writer.SetFileName(data_dir+"/depth1.png")
		print "writing" + data_dir+"/depth1.png"
		writer.SetInputData(vtk_depth_image)
		writer.Write()


		#tried a bunch  something i s wrong with link for m director render to my renderer oh well do by myself vertex map to noprmal map!!
		'''
		c++ depth buffer works
		python depth buffer through director doesnt work, trie pulling what i nedd out still nothing
		trie uaing render windows depth buffer fucntion o no luck,
		as for geting normals out of a scene also no luck too slow using raycast method
		gpu in the mix for converting normals transforming into normals ffro mscene and picking necessary normals

		ask greg about c++ version to do this all his code
		'''


		#vis.show image
		#might have to compute normals too
	
        #now get normal map and depth map and save each to same directory
        '''
        1.rgb
        2.depth
        3.normal map
        4.reflectance
        5.shading
        6.can we get raw ir screen from anywhere?
        '''
	showWindow = True
	if showWindow:
		application.app.start()
	else:
		consoleapp.ConsoleApp.start()