import fusion_server.fusion as fusion
import rospy

def main():
    image_capture = fusion.ImageCapture()
    ros_bag_filename = "/home/manuelli/spartan/sandbox/open3d/kuka_scene/fusion1519825352.8.bag"
    output_dir = "/home/manuelli/spartan/sandbox/open3d/kuka_scene/extracted_images"
    
    image_capture.load_ros_bag(ros_bag_filename)
    image_capture.process_ros_bag(image_capture.ros_bag, output_dir)
    # image_capture.start()
    # rospy.sleep(5.0)
    # image_capture.stop()
    # # image_capture.getTransforms()
    # num_depth_images = len(image_capture.depth_msgs)
    # num_rgb_images = len(image_capture.rgb_msgs)
    # print "received %d depth images" %(num_depth_images)
    # print "received %d rgb images" %(num_rgb_images)

    # image_capture.synchronize_rgb_and_depth_msgs()


if __name__ == "__main__":
    # rospy.init_node("image_capture_test")
    main()