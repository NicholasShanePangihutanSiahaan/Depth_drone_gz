import importlib.util
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('lidar_render_fix',ROOT/'tools/prepare_lidar_rendering.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)

def test_only_second_pass_sampler_modes_change_and_patch_is_idempotent():
    text = (module.SOURCE/module.MATERIAL).read_text()
    fixed = module.clamp_second_pass(text)
    assert module.clamp_second_pass(fixed)==fixed
    if text!=fixed:
        start = text.index('material GpuRaysScan2nd\n')
        assert fixed[:start]==text[:start]
        assert fixed.count('tex_address_mode clamp')-text.count('tex_address_mode clamp')==7
        assert fixed[start:].replace('        tex_address_mode clamp\n','')==text[start:]

def test_unknown_sampler_configuration_is_rejected():
    text = (module.SOURCE/module.MATERIAL).read_text()
    with pytest.raises(ValueError):
        module.clamp_second_pass(text.replace('texture_unit cubeUVTex','texture_unit unexpected'))
