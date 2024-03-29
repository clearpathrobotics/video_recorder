^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package video_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.10 (2024-01-04)
-------------------
* Add the mount_path argument & parameter to the vidio recorder node to allow remapping the result directory if e.g. this node is run inside a docker container
* Contributors: Chris Iverach-Brereton

0.0.9 (2023-09-07)
------------------
* Don't abort if we get a double-record or double-stop; treat these as successesful action invocation (but with .success set to false to indicate no new recording was made) and send the existing path if appropriate.
* Contributors: Chris Iverach-Brereton

0.0.8 (2023-08-23)
------------------

0.0.7 (2023-08-23)
------------------

0.0.6 (2023-08-16)
------------------
* Latch the `is_recording` topic
* Use dynamic subscribers to the video topic to preserve bandwidth
* Improve the default filename formatting
* Contributors: Chris Iverach-Brereton

0.0.5 (2023-04-13)
------------------
* Add ability to record meta-data about each recording
* Add exception handling if the tf lookup fails
* Switch from Mat to UMat to allow OpenCL support
* Add a space after the zoom colon
* Include the ROS topic name in the metadata
* Update the readme, add preliminary zoom support
* Add first-draft support for recording the robot and camera poses whenever a video starts or image is saved
* Contributors: Chris Iverach-Brereton

0.0.4 (2022-09-22)
------------------
* Make sure the node is actually installed correctly
* Contributors: Chris Iverach-Brereton

0.0.3 (2022-09-01)
------------------
* Allow cancelling fixed-duration videos
* Create the output directory if it doesn't already exist
* Contributors: Chris Iverach-Brereton

0.0.2 (2022-06-24)
------------------
* Add an optional parameter to set a hard maximum duration on videos recorded by the node.
* Contributors: Chris Iverach-Brereton

0.0.1 (2022-06-22)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton
