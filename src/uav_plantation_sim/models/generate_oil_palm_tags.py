#!/usr/bin/env python3

import shutil
from pathlib import Path

# ===== UBAH SESUAI LOKASI PROJECT =====
MODELS_DIR = Path.home() / "ProjekAtaka/gazebo_sim/src/uav_plantation_sim/models"

SOURCE_MODEL = MODELS_DIR / "oil_palm"

for tag_id in range(4):

    dst = MODELS_DIR / f"oil_palm_tag{tag_id}"

    # Hapus jika sudah ada
    if dst.exists():
        shutil.rmtree(dst)

    # Copy seluruh model oil_palm
    shutil.copytree(SOURCE_MODEL, dst)

    sdf_file = dst / "model.sdf"

    with open(sdf_file, "r") as f:
        sdf = f.read()

    # Ganti nama model
    sdf = sdf.replace(
        '<model name="oil_palm">',
        f'<model name="oil_palm_tag{tag_id}">'
    )

    # Visual AprilTag
    tag_visual = f"""
      <visual name="apriltag">

        <pose>0.226 0 1.35 0 1.57079632679 0</pose>

        <geometry>
          <plane>
            <size>0.25 0.25</size>
          </plane>
        </geometry>

        <material>

          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>

          <pbr>
            <metal>
              <albedo_map>
                materials/textures/tag25h9-{tag_id}.png
              </albedo_map>
            </metal>
          </pbr>

        </material>

      </visual>
"""

    # Sisipkan tepat setelah visual batang
    marker = "</visual>"

    idx = sdf.find(marker)

    if idx == -1:
        raise RuntimeError("Visual batang tidak ditemukan.")

    idx += len(marker)

    sdf = sdf[:idx] + tag_visual + sdf[idx:]

    with open(sdf_file, "w") as f:
        f.write(sdf)

    # Copy materials dari model apriltag
    src_material = MODELS_DIR / f"apriltag{tag_id}" / "materials"

    if src_material.exists():
        dst_material = dst / "materials"

        if dst_material.exists():
            shutil.rmtree(dst_material)

        shutil.copytree(src_material, dst_material)

    # Copy model.config bila belum ada
    src_cfg = SOURCE_MODEL / "model.config"

    if src_cfg.exists():
        shutil.copy(src_cfg, dst / "model.config")

print("Selesai.")
print("Model yang dibuat:")
for i in range(4):
    print(f"  oil_palm_tag{i}")