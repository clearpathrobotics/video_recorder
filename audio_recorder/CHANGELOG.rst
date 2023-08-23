^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
