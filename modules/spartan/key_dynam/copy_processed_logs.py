from __future__ import print_function

import argparse
import os
import shutil
from zipfile import ZipFile

def zip_and_copy():

    # src_dir = "/home/manuelli/data/key_dynam/hardware_experiments/logs/push_box_good"
    # dst_dir = "/media/manuelli/Extreme SSD/key_dynam/hardware_experiments/push_box_hardware"

    src_dir = "/media/manuelli/Extreme SSD/key_dynam/hardware_experiments/push_box_hardware"
    dst_dir = "/home/manuelli/data/key_dynam/dataset/push_box_hardware"


    logs_list = os.listdir(src_dir)
    logs_list.sort()


    for log_name in logs_list:
        if os.path.exists(os.path.join(dst_dir, log_name)):
            print("already copied log %s, skipping" %(log_name))
            continue
        else:
            print("processing log: %s" %(log_name))

        processed_dir = os.path.join(src_dir, log_name, "processed")
        processed_zip = os.path.join(src_dir, log_name, "processed.zip")

        if not os.path.exists(processed_zip):
            print("making zip file")
            # base name should have been nothing . . .
            shutil.make_archive(base_name=processed_dir, format="zip", base_dir=processed_dir)
            print("processed_dir", processed_dir)

        # copy zip file over
        dst_zip = os.path.join(dst_dir, log_name, "processed.zip")
        print("copying %s:%s" %(processed_zip, dst_zip))

        os.makedirs(os.path.dirname(dst_zip))
        shutil.copyfile(processed_zip, dst_zip)


def unzip_logs():
    logs_dir = "/home/manuelli/data/key_dynam/dataset/push_box_hardware"
    logs_list = os.listdir(logs_dir)
    logs_list.sort()

    for log_name in logs_list:
        tmp_dir = os.path.join(logs_dir, log_name, 'tmp')
        processed_dir = os.path.join(logs_dir, log_name, "processed")
        processed_zip = os.path.join(logs_dir, log_name, "processed.zip")

        if os.path.isdir(processed_dir):
            print("Already extracted, skipping: %s" %(log_name))
            continue
        else:
            print("Extracting log: %s" %(log_name))

        # see https://stackoverflow.com/questions/3451111/unzipping-files-in-python
        with ZipFile(processed_zip, 'r') as zf:
            zf.extractall(tmp_dir)
            zf.close()

        # basename = "media/manuelli/Extreme SSD/key_dynam/hardware_experiments/push_box_hardware"
        basename = "home/manuelli/data/key_dynam/hardware_experiments/logs/push_box_good"

        src = os.path.join(logs_dir, log_name, 'tmp', basename, log_name, 'processed')
        dst = processed_dir
        # print("moving %s:%s" %(src,dst))
        shutil.move(src, dst)
        shutil.rmtree(os.path.join(logs_dir, log_name, 'tmp'))

        # just try one to start




if __name__ == "__main__":
    # zip_and_copy()
    unzip_logs()



