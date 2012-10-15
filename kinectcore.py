from Queue import Queue
import freenect
import logging
import numpy
import threading

from config import INVERT_KINECT


class StreamerDied(Exception):
    pass


class OneQueue(object):

    def __init__(self):
        self.val = None
        self.event = threading.Event()
        self.lock = threading.Lock()

    def get(self):
        self.event.wait()
        with self.lock:
            self.event.clear()
            if isinstance(self.val, Exception):
                raise self.val
            else:
                return self.val

    def put(self, val):
        with self.lock:
            self.event.set()
            self.val = val


class KinectConsumer(object):

    def __init__(self, remove, decimate=1):
        self.remove = remove
        self.decimate = decimate
        self.queue = OneQueue()
        self.active = True

    def __iter__(self):
        return self

    def next(self):
        return self.queue.get()

    def stop(self):
        if self.active:
            self.remove(self.queue)
            self.active = False


class KinectProducer(object):

    def __init__(self, dev, device_num, stream_type, streamer):
        self.dev = dev
        self.device_num = device_num
        self.stream_type = stream_type
        self.streamer = streamer
        self.consumers = set() # set([KinectConsumer])
        self.lock = threading.Lock()
        self.frame = 0 # frame counter
        self.producing = False # true if freenect is sending us data

        assert stream_type in ('depth', 'video')

        if stream_type == 'depth':
            freenect.set_depth_mode(dev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_11BIT)
            freenect.set_depth_callback(dev, self._data_cb)
        elif stream_type == 'video':
            freenect.set_video_mode(dev, freenect.RESOLUTION_MEDIUM, freenect.VIDEO_RGB)
            freenect.set_video_callback(dev, self._data_cb)

    def stop(self):
        for consumer in set(self.consumers):
            consumer.queue.put(StreamerDied("Stream stopped"))
            consumer.stop()
        freenect.close_device(self.dev)

    def _data_cb(self, dev, data, timestamp):
        if INVERT_KINECT:
            data = data[::-1, ::-1] # Flip upside down

        with self.lock:
            for consumer in self.consumers:
                if self.frame % consumer.decimate == 0:
                    consumer.queue.put(numpy.copy(data))

        self.frame += 1

    def new_consumer(self, decimate):
        with self.lock:
            consumer = KinectConsumer(self._remove_consumer, decimate)
            if not self.consumers:
                self.streamer.add_command(lambda : self._update_stream('start'))
            self.consumers.add(consumer)
            return consumer

    def _remove_consumer(self, consumer):
        with self.lock:
            self.consumers.discard(consumer)
            if not self.consumers:
                self.streamer.add_command(lambda : self._update_stream('stop'))

    def _update_stream(self, action):
        logging.info("%s %s for %s" % (action, self.stream_type, self.device_num))
        getattr(freenect, action + '_' + self.stream_type)(self.dev)
        self.producing = action == 'start'


class KinectStreamer(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, name="KinectStreamer")
        self.producers = {} # (device_num, stream_type) -> KinectProducer
        self.command_q = Queue() # [callable] commands to run in streamer thread
        self.lock = threading.Lock()
        self.initialized = threading.Event()
        self.keep_running = False
        self.devs = [] # [DevPtr]

    num_devices = property(lambda self : len(self.devs))

    def new_consumer(self, stream_type, device_num=0, decimate=1):
        with self.lock:
            if self.keep_running:
                return self.producers[device_num, stream_type].new_consumer(decimate)

    def set_led(self, ledstate):
        self.add_command(lambda : [freenect.set_led(dev, ledstate) for dev in self.devs])

    def add_command(self, callback):
        self.command_q.put(callback)

    def run(self):
        try:
            ctx = freenect.init()

            for index in xrange(freenect.num_devices(ctx)):
                self.devs.append(freenect.open_device(ctx, index))

            for device_num, dev in enumerate(self.devs):
                for stream_type in ('video', 'depth'):
                    self.producers[device_num, stream_type] = \
                        KinectProducer(dev, device_num, stream_type, self)

            self.initialized.set()

            while self.keep_running:
                while not self.command_q.empty():
                    self.command_q.get()()

                if self._should_runloop():
                    freenect.base_runloop(ctx, self._body)
                else:
                    self.command_q.get()()
        finally:
            with self.lock:
                self.keep_running = False
                for producer in self.producers.itervalues():
                    producer.stop()
                self.producers = {}
            freenect.shutdown(ctx)
            self.devs = []
            self.initialized.set()

    def _should_runloop(self):
        return self.keep_running \
            and any(p.producing for p in self.producers.itervalues())

    def _body(self, ctx):
        if self._should_runloop():
            while not self.command_q.empty():
                self.command_q.get()()
        else:
            raise freenect.Kill()

    def start(self):
        if self.is_alive():
            return
        with self.lock:
            if not self.keep_running:
                self.keep_running = True
                threading.Thread.start(self)
                logging.info("Kinect streamer started")

    def stop(self):
        if not self.is_alive():
            return
        with self.lock:
            if self.keep_running:
                self.keep_running = False
                self.add_command(lambda : None)
        self.join()
        logging.info("Kinect streamer stopped")
