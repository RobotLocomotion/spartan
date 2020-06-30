from __future__ import print_function

import argparse
import os
import shutil

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument("--logs_dir", type=str, required=True)
    # parser.add_argument("--dest_dir", type=str, required=True)
    args = parser.parse_args()



    # logs_dir = args.logs_dir
    logs_dir = "/home/manuelli/data/key_dynam/hardware_experiments/logs/push_box_good"
    logs_list = os.listdir(logs_dir)
    logs_list.sort()
    dst_dir = "/media/manuelli/Extreme SSD/key_dynam/hardware_experiments/push_box_hardware"
    # dst_dir = args.dest_dir

    for log_name in logs_list:
        if os.path.exists(os.path.join(dst_dir, log_name)):
            print("already copied log %s, skipping" %(log_name))
            continue

        processed_dir = os.path.join(logs_dir, log_name, "processed")
        processed_zip = os.path.join(logs_dir, log_name, "processed.zip")

        if not os.path.exists(processed_zip):
            print("making zip file")
            shutil.make_archive(base_name=processed_dir, format="zip", base_dir=processed_dir)
            print("processed_dir", processed_dir)

        # copy zip file over
        dst_zip = os.path.join(dst_dir, log_name, "processed.zip")
        print("copying %s:%s" %(processed_zip, dst_zip))

        os.makedirs(os.path.dirname(dst_zip))
        shutil.copyfile(processed_zip, dst_zip)



