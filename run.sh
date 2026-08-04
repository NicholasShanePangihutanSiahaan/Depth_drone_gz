cd ~/ProjekAtaka/gazebo_sim
source /opt/ros/humble/setup.bash

echo "=== Memindahkan backup keluar dari workspace ==="

mkdir -p ~/ProjekAtaka/beehive_backups

find "$PWD/src" \
  -maxdepth 3 \
  -type d \
  -name 'beehive_drone_backup_*' \
  -print0 |
while IFS= read -r -d '' BACKUP_DIR; do
    BACKUP_NAME="$(basename "$BACKUP_DIR")"
    DEST="$HOME/ProjekAtaka/beehive_backups/${BACKUP_NAME}_$(date +%s)"

    echo "Memindahkan:"
    echo "  $BACKUP_DIR"
    echo "ke:"
    echo "  $DEST"

    mv "$BACKUP_DIR" "$DEST"
done

echo
echo "=== Mencari source package aktif ==="

PKG_XML="$(
    grep -RIl \
      --include='package.xml' \
      '<name>beehive_drone</name>' \
      "$PWD/src" |
    grep -v '_backup_' |
    head -n 1
)"

if [ -z "$PKG_XML" ]; then
    echo "ERROR: package.xml beehive_drone tidak ditemukan."
    exit 1
fi

PKG_DIR="$(dirname "$PKG_XML")"

echo "Package aktif: $PKG_DIR"

echo
echo "=== Memeriksa apakah source revisi sudah tertulis ==="

grep -n "takeoff_latched" \
  "$PKG_DIR/beehive_drone/flight_manager.py" || true

grep -n "PRESTREAM" \
  "$PKG_DIR/beehive_drone/mission_state_machine.py" |
  head || true

if ! grep -q "takeoff_latched" \
    "$PKG_DIR/beehive_drone/flight_manager.py"; then
    echo
    echo "ERROR: flight_manager.py masih versi lama."
    echo "Jalankan kembali revisi_total_beehive.sh terlebih dahulu."
    exit 2
fi

if ! grep -q 'transition("PRESTREAM"' \
    "$PKG_DIR/beehive_drone/mission_state_machine.py"; then
    echo
    echo "ERROR: mission_state_machine.py masih versi lama."
    echo "Jalankan kembali revisi_total_beehive.sh terlebih dahulu."
    exit 3
fi

echo
echo "=== Menambah waktu stabilisasi SITL ==="

sed -i -E \
  's/^([[:space:]]*prestream_sec:).*/\1 20.0/' \
  "$PKG_DIR/config/mission.yaml"

grep -n "prestream_sec" \
  "$PKG_DIR/config/mission.yaml"

echo
echo "=== Menghapus build dan instalasi lama ==="

rm -rf \
  build/beehive_drone \
  install/beehive_drone

find "$PKG_DIR" \
  -type d \
  -name '__pycache__' \
  -prune \
  -exec rm -rf {} +

echo
echo "=== Memeriksa duplikasi package ==="

colcon list | grep '^beehive_drone'

PACKAGE_COUNT="$(
    colcon list |
    awk '$1 == "beehive_drone" {count++} END {print count+0}'
)"

if [ "$PACKAGE_COUNT" -ne 1 ]; then
    echo "ERROR: ditemukan $PACKAGE_COUNT package beehive_drone."
    echo "Harus tepat satu package."
    exit 4
fi

echo
echo "=== Build ulang ==="

colcon build \
  --symlink-install \
  --packages-select beehive_drone \
  --event-handlers console_direct+

source install/setup.bash

echo
echo "=== Verifikasi Python module yang digunakan ==="

python3 - <<'PY'
import inspect

import beehive_drone.flight_manager as flight_manager
import beehive_drone.mission_state_machine as mission_state_machine

flight_file = inspect.getfile(flight_manager)
mission_file = inspect.getfile(mission_state_machine)

flight_source = inspect.getsource(flight_manager.FlightManager)
mission_source = inspect.getsource(
    mission_state_machine.MissionStateMachine
)

print("Flight manager :", flight_file)
print("Mission FSM    :", mission_file)

assert "takeoff_latched" in flight_source, (
    "flight_manager yang terpasang masih versi lama"
)

assert "PRESTREAM" in mission_source, (
    "mission_state_machine yang terpasang masih versi lama"
)

print("VERIFIKASI BERHASIL: revisi aktif.")

