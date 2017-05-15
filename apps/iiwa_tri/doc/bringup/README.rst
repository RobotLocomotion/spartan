Bringup Instructions
====================

.. contents:: :local:

Quick Links
----

-  `Mocap Bringup <#mocap>`__
-  `Smart Pad Bringup <#pad>`__
-  `Director / Drake Bringup <#drake>`__

Access
------
This document is confidential. Please do not release to the public.

Authors
-------

-  Sam Creasey
-  Eric Cousineau
-  Allison Fastman

Scope
-----
This document is intended for bringing up the IIWA arms for use on the
``roberto`` machine, interacting with mocap on ``frankie``.

Definitions
----

-  **Control Cabinet, Controller**: Offboard controller for the physical manipulator, connected to the host / client computer.
-  **Client, Remote Computer**: This is the host computer that will be connecting to the Control Cabinet.
-  **KUKA Smart Pad**: Control pad

   -  *Modes*:

      .. _definitions-aut:

      -  **AUT**: Autonomous / Automatic mode

         -  Necessary to run Drake controllers
         -  CAUTION: This does not have the same safety restrictions as
            **T1**. Please take care when using this mode and be vigilant with the E-Stop(s).

      -  **T1**: Out-of-the-box KUKA mode for manual control.

         -  Restrictions on joint compliance, some force thresholds, etc.

      -  **T2**: For... uh... other things

   -  *Actions*:

      -  **Mastering**: This does a minor runtime re-calibration of the joint and will return it to its home position.

Bringup Checklist
-----          

#. Wear safety glasses.

#. At the Control Cabinet:

   #. Turn on Control Cabinet with the soft power switch:

      .. image:: ./img/control_cabinet_power_switch.jpg
         :width: 400pt

   .. _cabinet-addresses:

   #. Review the IP addresses and ports on top of the Control Cabinet(s) for future reference:

      .. image:: ./img/control_cabinet_addresses.jpg
         :height: 400pt

.. _mocap:

#. If using the Motion Capture system:

   #. Plug power cable into the back of the Netgear Switch:

      .. image:: ./img/mocap_network_switch.jpg
         :width: 400pt

      -  You should see green lights on the back of the Optitrack Cameras, and traffic indicators on the switch ports

   #. At the Computer:

      #. Computer Hostname: ``frankie`` (should be running Windows)
      #. Wait until all cameras have booted (should take a couple of minutes)
      #. Once the cameras are booted, open the MOTIVE Software (on desktop)
      #. Open the most recent project to get the last set of calibrations and the object library:

         .. image:: ./img/mocap_motive_startup.png
            :width: 400pt

         -  You should see resolved Rigid Bodies

      #. At this point, you should see blue lights on the front of the Optitrack Cameras.

.. _pad:

#. At the KUKA Smart Pad

   .. image:: ./img/smart_pad.jpg
      :height: 200pt

   .. _manual:

   #. If you need to manually move the robot via the Smart Pad:

      .. _pad-config:

      #. Flip the knob the Configuration Mode:

         .. image:: ./img/smart_pad_mode_config.jpg
            :height: 400pt

      #. Change from **AUT** to **T1** (see picture and
         `Definitions`_ above)

      -  Click on the **Station** Menu in the top-right:

         .. image:: ./img/smart_pad_manual_options.jpg
            :width: 400pt

         -  This should indicate that you have **T1** selected.

      #. Ensure that **LBR\_iiwa\_14\_R...** is selected
      #. Flip the knob back to a locked mode
      #. Choose either **Joint Position** or **Mastering**

         -  **Joint Position**:

            #. Note that there are joints **A1** - **A7**
            #. You may move the joints with the **+** / **-** buttons on the right side of the pad.

               .. _deadman:

               #. Ensure that you have the deadman switches depressed:

                  .. image:: ./img/smart_pad_deadman_switch.jpg
                     :height: 300pt

                  -  *NOTE*: If you do not press these, the motion switches will not be enabled.
                  -  If you press them too hard, they will act as an E-Stop.

         -  **Mastering** (see `definitions <#definitions>`__ for a shallow explanation): *TROUBLESHOOTING HELPER*: Refer to this mode if you encounter any error message referring to any of the **A[X]** joints.

            #. For a given joint, press **Unmaster** if you wish to
               "re-calibrate" the joint.
            #. Select **Master**, and the joint should move to its effective home position, wiggle around, then move back.

   .. _aut:

   #. For Autonomous Mode (e.g., controlling the arm with ``drake``):

      .. RST doesn't support nested formatting???

      #.  Please review the **AUT** warning (see `here <#definitions-aut>`__)

      #. Flip the knob to allow configuration changes (see
         `here <#pad-config>`__)
      #. Ensure that **AUT** mode is selected (flip the knob as mentioned)
      #. Ensure that **LBR_iiwa_14_R...** is selected
      #. Flip the knob back to lock the configuration
      #. Go to **Applications** at the top of the screen, find the ones that start with **Drake...**
      #. Select **Drake FRIPositionDriver**:

         .. image:: ./img/smart_pad_drake_position.jpg
            :width: 400pt

      #. Hit **Play**

         -  You will see the state go from **Selected** to **Running**
         -  You will see the IP address of the intended client computer running the ``drake`` code

      .. _drake:

      #. Next, run your ``drake`` / ``director`` / ``spartan`` behaviors:

         #. Computer Hostname: ``roberto``
         #. Execute the following ``bash`` commands in the terminal:

            :: use_spartan cd ~/sammy-demo/ bot-procman-sheriff -l iiwa_hardware_near.pmd  # Or _far for the far arm

            -  This will bring up ``procman``

         #. Start the following commands (right-click, **Start**):

            -  ``vision-drivers/optitrack-driver``
            -  ``iiwa_drivers`` - all (just right-click this top-level node)

               -  You should see ``FRI connection established`` on the Smart Pad

            -  ``schunk_driver``

               -  You will see the gripper open and close
               -  In ``procman``, you may see ``Non-success response 1``
                  - this is actually OK if the process status remains green / OK
               -  Refer to the ``procman`` command or the `cabinet IP address / ports <#cabinet-addresses>`__ for the Schunk IP address

                  -  *TROUBLESHOOTING HELPER*: If you open
                     ``http://${SCHUNK_IP_ADDRESS}/`` in your web browser, you will be able to view the web interface.

                     -  If you encounter an issue with the gripper, this may be due to a driver fault / miscommunication:

                        #. Click **Stop**, then **Ack**

            -  ``openni-driver`` - Once this is running, it should same something to the effect of ``First color image from device received``

               -  This is necessary for performing Logging (see `here <#logging>`__).

            -  ``director`` - the main GUI

               #. Try panning around and viewing the markers
               #. Click **Task Panel**. You can set the **Base rigid body** to **Kuka Base (Near\|Far)** (depending on your robot)
               #. Set **Target rigid body** to **Black Box**
               #. Iterate through the actions

                  -  Select the top-level task, and press **continue**

               #. Set **Place target** coordinates accordingly. You may need to change the z-location of the targets.
               #. Click **pause** if something goes awry
               #. If you need to change the robot pose / joints in
                  **T1** mode:

                  #. Go to the ``director`` window
                  #. Go to the **Teleop Panel**
                  #. Switch to **IK mode** or **Joint Teleop mode**

                     -  *Insert more instructions later...*

            .. _logging:

            -  Logging:

               -  Script: ``lcm-logger``
               -  Plug in Asus Xtion if recording experiment data (for later use on perception analysis)
               -  Can record optitrack data and play it back for use on a simulation machine

Shutdown Checklist
----

#. Close ``procman`` (this should close all dependent process)

   -  You should hear a *PING* when the IIWA brakes engage. This happens due to a watchdog on the controller side, which will engage the brakes if it hasn't received communication from the driver within a short timeframe. (effectively once you've lifted your finger from the keyboard)

#. Close **MOTIVE**
#. Turn off the Control Cabinet soft power switch
#. Unplug the Netgear switch

   -  CAUTION: Ensure that you have closed **MOTIVE** before you unplug the Mocap system. Failing to do so may lead to losing calibration data.

#. Go home.
