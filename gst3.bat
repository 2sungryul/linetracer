gst-launch-1.0 -v udpsrc port=8003 ! application/x-rtp, encoding-name=(string)H264, payload=(int)96 ! rtph264depay ! queue ! avdec_h264 ! videoconvert! autovideosink