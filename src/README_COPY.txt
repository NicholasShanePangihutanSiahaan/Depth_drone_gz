FILE YANG DIGANTI
==================
1. beehive_drone/beehive_drone/mission_analyzer.py
2. Tambahkan isi beehive_drone/config/ANALYZER_YAML_BLOCK.txt ke bagian bawah mission.yaml

OUTPUT BARU
===========
Setiap misi langsung membuat folder:
~/beehive_mission_results/mission_YYYYMMDD_HHMMSS_PID/

Isi folder:
- trajectory_detailed.csv
- state_events.csv
- mission_metrics.csv
- trees_latest.csv
- state_durations.csv
- mission_summary.csv
- map_2d.png
- map_3d.png

CSV trajectory dibuat sejak node mulai dan setiap baris langsung di-flush ke disk.
CSV/PNG juga di-autosave setiap 10 detik, saat state berubah, saat DONE, dan saat shutdown.
