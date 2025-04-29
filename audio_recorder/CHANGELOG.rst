^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


2.0.2 (2025-04-29)
------------------

2.0.1 (2025-04-23)
------------------
* Add missing dependencies
* Contributors: Chris Iverach-Brereton

2.0.0 (2025-04-23)
------------------
* Update `audio_recorder` to Jazzy (`#7 <https://github.com/clearpathrobotics/video_recorder/issues/7>`_)
  * Update audio_recorder package to ROS 2
* Contributors: Chris Iverach-Brereton

0.1.1 (2024-06-03)
------------------
* Fix the timestamp assignment in the audio recorder
* Contributors: Chris Iverach-Brereton

0.1.0 (2024-05-27)
------------------
* Add a header to the result objects for all actions (`#6 <https://github.com/clearpathrobotics/video_recorder/issues/6>`_)
* Contributors: Chris Iverach-Brereton

0.0.10 (2024-01-04)
-------------------
* Add the mount_path argument & parameter to the audio recorder node to allow remapping the result directory if e.g. this node is run inside a docker container
* Contributors: Chris Iverach-Brereton

0.0.9 (2023-09-07)
------------------
* Don't abort if we get a double-record or double-stop; treat these as successesful action invocation (but with .success set to false to indicate no new recording was made) and send the existing path if appropriate.
* Contributors: Chris Iverach-Brereton

0.0.8 (2023-08-23)
------------------
* Add a dependency on alsa-utils
* Contributors: Chris Iverach-Brereton

0.0.7 (2023-08-23)
------------------
* Fix a typo in the audio recorder node
* Contributors: Chris Iverach-Brereton

0.0.6 (2023-08-16)
------------------
* Latch the `is_recording` topic
* Improve the default filename formatting
* Contributors: Chris Iverach-Brereton

0.0.5 (2023-04-13)
------------------
* Add ability to record meta-data about each recording
* Initial implementation of audio_recorder metadata to match the video_recorder implementation
* Create the output directory for the audio recorder node to keep the behaviour consistent with the video recorder
* Contributors: Chris Iverach-Brereton

0.0.4 (2022-09-22)
------------------

0.0.3 (2022-09-01)
------------------
* Update the docs, add the is_recording + default filename to the audio recorder
* Add the first-pass audio-recorder action server. Very similar format to the video recorder, but for dumping ALSA input to wav files.
* Contributors: Chris Iverach-Brereton

0.0.1 (2022-06-22)
------------------
