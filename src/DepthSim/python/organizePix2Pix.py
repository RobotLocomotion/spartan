import os
path = '/media/drc/DATA/chris_labelfusion/logs_test/test_cycle_gan/'
sim = path+'trainA/'
real = path+'trainB/'
sim_files = [f for f in os.listdir(sim) if os.path.isfile(sim+f)]
real_files = [f for f in os.listdir(real) if os.path.isfile(real+f)]

sim_files.sort()
real_files.sort()
for i in range(100):
	os.rename(sim+sim_files[i],sim+"test/"+real_files[i])
	os.rename(real+real_files[i],real+"test/"+real_files[i])
for i in range(100,200):
	os.rename(sim+sim_files[i],sim+"val/"+real_files[i])
	os.rename(real+real_files[i],real+"val/"+real_files[i])
for i in (200,3999):
	os.rename(sim+sim_files[i],sim+"train/"+real_files[i])
	os.rename(real+real_files[i],real+"train/"+real_files[i])