import os
import argparse
import yaml
import spartan.utils.utils as spartanUtils

parser = argparse.ArgumentParser()
parser.add_argument('--camera_name', default='xtion_pro')
parser.add_argument('--rgb', default=False, action='store_true')
parser.add_argument('--ir', default=False, action='store_true')
args = parser.parse_args()

def extract_intrinsics(path_to_tar_file, path_to_camera_info_yaml_file):
    cmd = 'mkdir -p /tmp/calibrationdata'
    os.system(cmd)
    cmd = 'tar xvf ' + path_to_tar_file + ' -C ' + '/tmp/calibrationdata' + ' > /dev/null 2>&1'
    os.system(cmd)

    camera_info_dict = spartanUtils.getDictFromYamlFilename('/tmp/calibrationdata/ost.yaml')

    print camera_info_dict


if __name__ == '__main__':
    path_to_camera_info_yaml_file = os.path.join(spartanUtils.getSpartanSourceDir(),
                 'src/catkin_projects/camera_config/data', args.camera_name, 'master')
    path_to_tar_file = '/tmp/calibrationdata.tar.gz'
    extract_intrinsics(path_to_tar_file, path_to_camera_info_yaml_file)
