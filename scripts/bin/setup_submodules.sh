#!/bin/bash
git submodule init
git submodule deinit src/Gyges
git rm src/Gyges
git submodule update