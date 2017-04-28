
outputFileName = 'output.vtp'

def foo():
    obj = om.findObjectByName('openni point cloud')
    pd = obj.polyData
    ioUtils.writePolyData(pd, outputFileName)
    print 'saved:', outputFileName
