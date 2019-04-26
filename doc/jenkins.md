CI with Jenkins
============================
CI is provided by Jenkins, presently running on a DRC laptop running Ubuntu
16.04 with nvidia-418.56, CUDA 10, and nvidia-docker2. Three Jenkins jobs test
our build:

- A nightly-plus-whenever-it-is-updated build of the master branch. Master is tested by following the build-and-test routine described below.

- A nightly-plus-whenever-it-is-updated build of the master branch, rebased on upstream Drake, tested by following the build-and-test routine described below.

- A whenever-it-is-requested build of PR branches, **which can be requested by including the phrase "Jenkins please test" in a comment on the PR**. The branches are tested by merging them into master (if possible) and then following the build-and-test routine described below. For now, tests can only be demanded by whitelists devs (`gizatt`, `manuelli`, `peteflorence`, and many others). Anyone else can *request* a test, but one of those admins will have to confirm to Jenkins that the test can be run. This feature uses [this tool](https://github.com/jenkinsci/ghprb-plugin) under the hood, so admins can use the command `ok to test` to accept a PR for testing, and `add to whitelist` to add the author of the PR to the whitelist forever.

Jenkins clones a completely fresh copy of the repository into a working directory,
run `git submodule update --init`, and then runs::

    python ${WORKSPACE}/setup/docker/docker_build.py
    python ${WORKSPACE}/setup/docker/docker_run.py --container "spartan-prs-merged" --no_udp --entrypoint "/home/jenkins/spartan/setup/docker/run_jenkins.sh"

If any step of this returns a nonzero error code, the build is considered failed.
That includes failures in initializing any submodule; errors provisioning or
launching a docker container; or errors detected by the `run_jenkins` script,
which contains its own error checking on the CMake configuration and the build,
as well as a battery of code tests.

Developer's guide to Spartan-Jenkins
==============================

The Spartan build server interface is available at http://spartan-jenkins.csail.mit.edu:8080/. Anyone can click around and view build logs. To tweak settings, a user must log in (button in the upper right). If you want permissions to do so, contact `gizatt` and I'll make you an account.

## Tweaking build settings

Click on any of the jobs (e.g. [`spartan-prs-merged`](http://spartan-jenkins.csail.mit.edu:8080/job/spartan-prs-merged/)) to get an overview of that job. On the left hand panel, you should see a "Configure" link that takes you to that job's control panel. (Remember to log in first.) E.g. [here](http://spartan-jenkins.csail.mit.edu:8080/job/spartan-prs-merged/configure) is the control panel for `spartan-prs-merged`. There is a ton of obscure settings in here -- the ones that you are most likely to need to update are:

- **Adding devs to the whitelist:** under "Build Triggers", you'll see a box filled with whitelisted usernames. Add folks as desired, then save + apply the settings.
- **Tweak the build command:** under "Build", there's a box that contains the magic bash commands used to actually invoke the build, including those `docker_build.py` and `docker_run.py` invocations listed above. Tweaking these lines tweaks what the build server does.

Most things are self-explanatory. Jenkins has its own SSH keys and github account so it can hook into Spartan.

## SSH access to spartan-jenkins

The build server can be ssh'd into, but it only supports passwordless entry into the `locomotion` user. That means you need to follow [these](https://www.debian.org/devel/passwordlessssh) directions before you can SSH in: in short, append your SSH public key to the `~/.ssh/authorized_keys` file on Jenkins. You'll need to physically access Jenkins to do this.

### How to poke around failed builds

`sudo su jenkins` to become the `jenkins` user and then `cd ~/workspace`. The workspace for each job is in an appropriately named subfolder. Note that each job only has one workspace that gets cleared for each new build, so if you need to debug a build, it's probably wise to pause Jenkins. (Pause with the [`quietDown`](http://spartan-jenkins.csail.mit.edu:8080/quietDown) command, unpause with the [`cancelQuietDown`](http://spartan-jenkins.csail.mit.edu:8080/cancelQuietDown) command.)