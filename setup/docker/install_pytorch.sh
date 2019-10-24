#!/bin/bash

set -euxo pipefail

# The pytorch
# pip install https://download.pytorch.org/whl/cu100/torch-1.1.0-cp27-cp27mu-linux_x86_64.whl
pip install torch torchvision

# The open3d
pip install open3d-python==0.5.0.0

# For maskrcnn, note that maskrcnn should be installed after container lanunching
pip install pycocotools
pip install ninja yacs cython matplotlib tqdm typing