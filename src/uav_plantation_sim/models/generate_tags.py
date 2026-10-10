import shutil
from pathlib import Path

TEMPLATE = Path("apriltag_template")

NUM_TAGS = 135

template_sdf = (TEMPLATE / "model.sdf").read_text()

template_config = (TEMPLATE / "model.config").read_text()

for i in range(NUM_TAGS):

    dst = Path(f"apriltag{i}")

    if dst.exists():
        shutil.rmtree(dst)

    shutil.copytree(TEMPLATE, dst)

    # ganti nama model
    sdf = template_sdf.replace(
        'apriltag_template',
        f'apriltag{i}'
    )

    sdf = sdf.replace(
        "TEXTURE_FILE",
        f"tag25h9-{i}.png"
    )

    (dst / "model.sdf").write_text(sdf)

    config = template_config.replace(
        "apriltag_template",
        f"apriltag{i}"
    )

    (dst / "model.config").write_text(config)

print("Done.")