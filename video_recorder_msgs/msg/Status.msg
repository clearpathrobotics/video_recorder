uint8 WAITING=1
uint8 RUNNING=2
uint8 RECORDING=4
uint8 PHOTO_TIMER=8

# A bit-field indicating the current state of the node
# See constants above for each bit
uint8 status

# The raw count of frames received from the camera in the last 1s
uint32 frames_received_last_second

# The number of frames written to video/image files in the last second
# May be zero if we're not actually recording right now
uint32 frames_processed_last_second
