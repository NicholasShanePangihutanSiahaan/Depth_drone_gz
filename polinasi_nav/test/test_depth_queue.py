from polinasi_nav.depth_queue import DepthTransformQueue


def test_repeated_new_image_before_TF_does_not_starve_ready_older_frame():
    queue=DepthTransformQueue(6);after=-1.
    for n in range(20):
        now=20.+n*.2
        queue.push(now,now,now)
        # One image period of TF delay, with a new image arriving first.
        ready=queue.select(now+.01,1.,after,lambda stamp: stamp if stamp<=now-.1 else None)
        if n==0:assert ready is None;continue
        message,_=ready
        assert abs(message-(now-.2))<1e-9
        assert message>after
        after=message;queue.acknowledge(message)
        assert queue.status()['pending']==1
    assert queue.tf_waits>0 and queue.expired_drops==0


def test_stale_or_already_submitted_frames_never_refresh_freshness():
    queue=DepthTransformQueue();queue.push(1.,'stale',1.);queue.push(2.,'old',2.)
    assert queue.select(2.1,1.,2.,lambda msg:True) is None
    assert queue.obsolete_drops==2
    queue.push(3.,'expired',3.)
    assert queue.select(4.,1.,2.,lambda msg:True) is None
    assert queue.expired_drops==1


def test_bounded_overflow_out_of_order_duplicate_and_newest_ready_selection():
    queue=DepthTransformQueue(3)
    for stamp in (3.,1.,2.,4.):queue.push(stamp,stamp,4.)
    assert queue.overflow_drops==1
    assert not queue.push(4.,'duplicate',4.)
    selected=queue.select(4.1,3.,0.,lambda stamp: 'exact' if stamp<=3. else None)
    assert selected==(3.,'exact')
    queue.acknowledge(3.)
    assert queue.status()['pending']==1 and queue.status()['newest_stamp']==4.


def test_no_transform_and_invalid_frame_do_not_commit_new_timestamps():
    queue=DepthTransformQueue();queue.push(2.,'bad',2.);queue.push(2.2,'good',2.2)
    assert queue.select(2.3,1.,0.,lambda msg:None) is None
    assert queue.selected_stamp is None
    queue.discard(2.)
    assert queue.select(2.3,1.,0.,lambda msg:'tf')==('good','tf')
    assert queue.status()['selected_stamp']==2.2


def test_transform_lookup_uses_original_frame_stamp_not_current_time():
    queue=DepthTransformQueue();queue.push(20.,dict(stamp=20.),20.1)
    queue.push(20.2,dict(stamp=20.2),20.3)
    looked=[]
    def lookup(message):
        looked.append(message['stamp'])
        return (message['stamp'],message['stamp']) if message['stamp']<=20.1 else None
    frame,transforms=queue.select(20.4,1.,-1.,lookup)
    assert looked==[20.2,20.] and frame['stamp']==20. and transforms==(20.,20.)
    assert queue.selected_queue_wait==20.4-20.1
