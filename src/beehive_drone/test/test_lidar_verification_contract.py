from types import SimpleNamespace
from beehive_drone.mission_state_machine import MissionStateMachine


def fsm():
    return SimpleNamespace(state='VERIFY_TREE',target_tree=SimpleNamespace(id=7,x=3.,y=2.),lidar_result=None,adaptive_estimates={},get_parameter=lambda name:SimpleNamespace(value=True))

def test_camera_height_cannot_replace_lidar_height():
    node=fsm()
    MissionStateMachine.trunk_estimate_cb(node,SimpleNamespace(data=[7,0,7]))
    assert node.adaptive_estimates=={}

def test_only_selected_tree_can_be_verified():
    node=fsm()
    MissionStateMachine.lidar_result_cb(node,SimpleNamespace(data=[8,3,2,0,4,.3]))
    assert node.lidar_result is None
    MissionStateMachine.lidar_result_cb(node,SimpleNamespace(data=[7,3.1,2.1,.2,4,.3]))
    assert node.lidar_result[:5]==(7,3.1,2.1,.2,4)
    assert node.adaptive_estimates[7][:2]==(.2,4)

def test_far_away_cylinder_cannot_verify_phantom_candidate():
    node=fsm()
    MissionStateMachine.lidar_result_cb(node,SimpleNamespace(data=[7,6,2,0,4,.3]))
    assert node.lidar_result is None
