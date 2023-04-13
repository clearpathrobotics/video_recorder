^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package video_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
