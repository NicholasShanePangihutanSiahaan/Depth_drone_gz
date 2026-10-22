"""Bounded, nonblocking stamped camera queue awaiting exact-time transforms."""
import math


class DepthTransformQueue:
    def __init__(self,capacity=6):
        if int(capacity)!=capacity or capacity<1:raise ValueError('invalid queue capacity')
        self.capacity=int(capacity);self.frames=[]
        self.overflow_drops=0;self.expired_drops=0;self.obsolete_drops=0;self.tf_waits=0
        self.selected_stamp=None;self.selected_queue_wait=None
        self.selected_receive_delay=None;self.selected_data_age=None;self.superseded_drops=0

    def push(self,stamp,message,received_at):
        if not math.isfinite(stamp) or any(row[0]==stamp for row in self.frames):return False
        self.frames.append((float(stamp),message,float(received_at)))
        self.frames.sort(key=lambda row:row[0])
        while len(self.frames)>self.capacity:self.frames.pop(0);self.overflow_drops+=1
        return True

    def select(self,now,timeout,after,lookup):
        retained=[]
        for row in self.frames:
            stamp=row[0]
            if stamp<=after:self.obsolete_drops+=1
            elif not 0<=now-stamp<timeout:self.expired_drops+=1
            else:retained.append(row)
        self.frames=retained
        # Newest TRANSFORMABLE, not simply newest received. Pending newer
        # images remain queued so repeated one-tick TF lag cannot starve mapping.
        for stamp,message,received_at in reversed(self.frames):
            transforms=lookup(message)
            if transforms is None:self.tf_waits+=1;continue
            self.selected_stamp=stamp;self.selected_queue_wait=now-received_at
            self.selected_receive_delay=received_at-stamp;self.selected_data_age=now-stamp
            return message,transforms
        return None

    def discard(self,stamp):
        self.frames=[row for row in self.frames if row[0]!=stamp]

    def acknowledge(self,stamp):
        self.superseded_drops+=sum(row[0]<stamp for row in self.frames)
        self.frames=[row for row in self.frames if row[0]>stamp]

    def status(self):
        return dict(capacity=self.capacity,pending=len(self.frames),
            oldest_stamp=self.frames[0][0] if self.frames else None,
            newest_stamp=self.frames[-1][0] if self.frames else None,
            overflow_drops=self.overflow_drops,expired_drops=self.expired_drops,
            obsolete_drops=self.obsolete_drops,tf_wait_attempts=self.tf_waits,
            selected_stamp=self.selected_stamp,selected_queue_wait_seconds=self.selected_queue_wait,
            selected_receive_delay_seconds=self.selected_receive_delay,
            selected_acquisition_age_seconds=self.selected_data_age,superseded_drops=self.superseded_drops,
            policy='latest_exact_stamp_transformable_within_unchanged_freshness')
