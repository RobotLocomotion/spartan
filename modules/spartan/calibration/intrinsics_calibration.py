import os
import argparse
import yaml
import spartan.utils.utils as spartanUtils


def extract_intrinsics(path_to_tar_file, path_to_camera_info_yaml_file,
                       camera_name):
    cmd = 'mkdir -p /tmp/calibrationdata'
    os.system(cmd)
    cmd = 'tar xvf ' + path_to_tar_file + ' -C ' + '/tmp/calibrationdata' + ' > /dev/null 2>&1'
    os.system(cmd)

    camera_info_dict = spartanUtils.getDictFromYamlFilename('/tmp/calibrationdata/ost.yaml')

    print camera_info_dict

    camera_info_dict['camera_name'] = camera_name

    spartanUtils.saveToYaml(camera_info_dict, path_to_camera_info_yaml_file)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_name', default='xtion_pro')
    parser.add_argument('--rgb', default=False, action='store_true')
    parser.add_argument('--ir', default=False, action='store_true')
    args = parser.parse_args()

    camera_info_filename = ""
    if args.ir:
        camera_info_filename = 'depth_camera_info.yaml'
    elif args.rgb:
        camera_info_filename = 'rgb_camera_info.yaml'
    else:
        print "Need to pass either --rgb or --ir"
        exit(0)

    path_to_camera_info_yaml_file = os.path.join(spartanUtils.getSpartanSourceDir(),
                 'src/catkin_projects/camera_config/data', args.camera_name,
                 'master', camera_info_filename)

    path_to_tar_file = '/tmp/calibrationdata.tar.gz'
    extract_intrinsics(path_to_tar_file, path_to_camera_info_yaml_file,
                       args.camera_name)
