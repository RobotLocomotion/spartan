import os

os.system("cd ../src/ply && make")

path = os.getcwd() + "/../data/logs/pipeline-test/"
filename = "trimmed-log.lcmlog.ply"
#print path

os.system("../src/ply/ply2ascii <" + path + filename + "> " + path + "converted.ply")