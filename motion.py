#!/usr/bin/env python

import cv2
import logging
import numpy
import threading

import kinectcore

from config import VALID_THRESHOLD, Z_THRESHOLD, MOTION_THRESHOLD, LOST_THRESHOLD
from config import DECAY_K


def frame_to_depth(frame):
    mask = frame > 1070
    frame = frame.astype(numpy.float)

    # Calculate depth in meters as a function of depth value
    depth = 1.0 / (frame * -0.0030711016 + 3.3309495161)

    # Fill the masked (invalid) areas with "5 meters" for computational purposes
    depth = numpy.ma.filled(numpy.ma.array(depth, mask=mask), 5)

    return depth, mask

def depth_to_img(frame):
    return 255 - ((30 * frame).astype(numpy.uint8))

def delta_to_img(frame):
    return numpy.clip((60 * frame), 0, 255).astype(numpy.uint8)


class MotionSensor(threading.Thread):

    def __init__(self, kinect, device_num=0):
        threading.Thread.__init__(self, name="MotionSensor")
        self.kinect = kinect
        self.device_num = device_num
        self.debug = False
        self.detected = threading.Event()
        self.keep_running = True

    def run(self):
        self.detected.clear()

        stream = self.kinect.new_consumer('depth', device_num=self.device_num,
                                          decimate=5)

        # Load depth filter
        try:
            depth_filter = numpy.load("depth_filter.npy")
        except Exception:
            depth_filter = None

        # Drop initial frames that do not meet the valid threshold
        while numpy.count_nonzero(stream.next() != 2047) < VALID_THRESHOLD:
            if not self.keep_running:
                return

        # Drop a few more frames to ensure a stable image
        for i in range(30):
            stream.next()
            if not self.keep_running:
                return

        # Obtain reference image
        ref, mask = frame_to_depth(stream.next())

        # Apply depth filter
        if depth_filter is not None:
            ref = numpy.minimum(ref, depth_filter)

        # Blur it
        ref = cv2.GaussianBlur(ref, (0, 0), 2)

        # Create reference mask buffer
        ref_mask_buf = mask.astype(numpy.float)

        # Create dilation kernel
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))

        for frame in stream:
            # Get current frame
            depth, mask = frame_to_depth(frame)

            # Apply depth filter
            if depth_filter is not None:
                depth = numpy.minimum(depth, depth_filter)

            # Blur the depth
            depth = cv2.GaussianBlur(depth, (0, 0), 2)

            # Booleanize the current reference mask buffer
            ref_mask = ref_mask_buf > 0.5

            # Mask out invalid pixels in either image
            invalid = numpy.logical_or(mask, ref_mask)

            # Dilate the mask
            invalid = cv2.dilate(invalid.astype(numpy.uint8), dilate_kernel).astype(bool)

            # Count pixels lost vs. the reference image
            lost = numpy.logical_and(mask, numpy.logical_not(ref_mask))
            lost_count = numpy.count_nonzero(lost)

            # Mask both arrays
            masked_ref = numpy.ma.array(ref, mask=ref_mask)
            masked_depth = numpy.ma.array(depth, mask=invalid)

            # Compare, blur the difference
            delta = numpy.ma.filled(numpy.abs(masked_ref - masked_depth), 0)
            delta = cv2.GaussianBlur(delta, (0, 0), 1)

            # Mask out pixels under the threshold
            delta = numpy.ma.array(ref, mask=(delta < Z_THRESHOLD))

            # Compute the sum of deltas as a motion value
            motion = sum(sum(numpy.ma.filled(delta, 0)))

            # Accumulate into the reference buffer
            ref = ref * (1 - DECAY_K) + numpy.where(mask, ref, depth) * DECAY_K
            ref_mask_buf = ref_mask_buf * (1 - DECAY_K) + mask * DECAY_K

            # Trigger the alarm if motion or excessive lost pixels are detected
            if motion > MOTION_THRESHOLD or lost_count > LOST_THRESHOLD:
                if not self.detected.is_set():
                    logging.info("Motion detected (%d,%d)", motion, lost_count)
                self.detected.set()

            if self.debug:
                print motion > MOTION_THRESHOLD, lost_count > LOST_THRESHOLD
                cv2.imshow("Ref %d" % self.device_num, depth_to_img(masked_ref))
                cv2.imshow("Depth %d" % self.device_num, depth_to_img(masked_depth))
                cv2.imshow("Delta %d" % self.device_num, delta_to_img(numpy.ma.filled(delta, 0)))
                if cv2.waitKey(10) == 27:
                    return

            if not self.keep_running:
                return

    def start(self):
        if self.is_alive():
            return
        logging.info("Motion detection started")
        self.keep_running = True
        threading.Thread.start(self)

    def stop(self):
        if not self.is_alive():
            return
        self.keep_running = False
        self.join()
        logging.info("Motion detection stopped")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s')
    kinect = kinectcore.KinectStreamer()
    kinect.start()
    try:
        kinect.initialized.wait()
        if kinect.num_devices == 0:
            logging.error("no kinect devices")
        else:
            sensors = [MotionSensor(kinect, device_num)
                       for device_num in range(kinect.num_devices)]
            for sensor in sensors:
                sensor.debug = True
                sensor.run()
    finally:
        kinect.stop()
