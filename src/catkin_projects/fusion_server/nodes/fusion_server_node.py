import rospy
from fusion_server.fusion_server import FusionServer

if __name__ == "__main__":
    fs = FusionServer()
    fs.run_fusion_data_server()