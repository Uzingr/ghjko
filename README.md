# Step per avviare ghjko
1. Avviare il container all'interno della cartella ghjko:
```bash
  $ sudo docker compose build
  $ sudo docker compose up
```
3. Aprire un nuovo terminale e avviare il container
```bash
  $ sudo docker exec -it ghjko-bridge-1 bash
```
5. Entrare in ghjko/vicon_tracker_ros
6. Eseguire il nodo ros
```bash
  $ ros2 run vicon_tracker_ros vicon_tracker_client vicon_tracker_client.launch.py
```

