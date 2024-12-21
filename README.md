# `klaszterezes` csomag
ROS 2 Python csomag Lidar adatok klaszterezésére és vizualizációjára.  
[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Áttekintés
A `klaszterezes` csomag a `/scan` topikból származó Lidar adatokat dolgozza fel, azonosítja a klasztereket (objektumokat) és vonalakat, majd megjeleníti az eredményeket egy Tkinter-alapú grafikus felületen. Emellett a detektált objektumokat, vonalakat és a feldolgozott Lidar távolságokat külön ROS 2 topikokra publikálja további feldolgozás céljából.

### Funkciók
- **Klaszterek detektálása**: A Lidar pontokat közelség alapján csoportosítja.
- **Vonalak detektálása**: A Lidar adatokból folytonos szegmenseket (vonalakat) azonosít.
- **Vizualizáció**: A Tkinter vásznon megjeleníti a Lidar pontokat, klasztereket és vonalakat.
- **ROS 2 topikok**: A feldolgozott adatokat az alábbi topikokra publikálja:
  - `/lidar_objects` (klaszterek)
  - `/lidar_lines` (vonal szegmensek)
  - `/lidar_ranges` (feldolgozott Lidar távolságok).

---

## Telepítés és build

Feltételezzük, hogy a munkakönyvtár `~/ros2_ws/`.

### A csomag klónozása
```bash
cd ~/ros2_ws/src
git clone https://github.com/adamoravecz/klaszterezes
```

### A csomag buildelése
```bash
cd ~/ros2_ws
colcon build --packages-select klaszterezes --symlink-install
```

<details>
<summary>Ne felejtse el a ROS környezetet forrásolni a parancsok előtt!</summary>

```bash
source ~/ros2_ws/install/setup.bash
```
</details>

---

## Használat

### A csomag indítása
A `klaszterezes` csomagot az alábbi launch fájl segítségével indítható:
```bash
ros2 launch klaszterezes launch_example1.launch.py
```

### Feliratkozott topikok
- `/scan` (`sensor_msgs/msg/LaserScan`): Lidar szkennelési adatok.

### Publikált topikok
- `/lidar_objects` (`std_msgs/msg/Int32MultiArray`): Klaszterek méreteinek listája.
- `/lidar_lines` (`std_msgs/msg/Int32MultiArray`): Vonal szegmensek hosszainak listája.
- `/lidar_ranges` (`std_msgs/msg/Float32MultiArray`): Feldolgozott Lidar távolságok.

---

## Vizualizáció
A csomag futtatása közben egy Tkinter ablak jelenik meg, amely az alábbiakat mutatja:
- **Pontok**: A Lidar szkennelési pontokat jeleníti meg, klaszterek szerint színezve.
- **Vonalak**: A Lidar adatokból detektált folytonos szegmenseket (vonalakat) rajzolja ki.

---

## Tesztelés
A csomag teszteléséhez a [turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim.html) csomagot használtam, amely egy Lidar-szerű környezetet szimulál. A következő lépésekkel tesztelhető a csomag:

### Turtlesim telepítése
```bash
sudo apt install ros-humble-turtlesim
```

### Turtlesim indítása
```bash
ros2 run turtlesim turtlesim_node
```

### Lidar szimuláció futtatása
Publikáljon dummy Lidar adatokat a `/scan` topikra egy szkripttel vagy más ROS 2 csomaggal, majd figyelje meg az eredményeket a Tkinter ablakban.

### Publikált topikok ellenőrzése
A feldolgozott adatokat ellenőrizhetők az alábbi parancsokkal:
```bash
ros2 topic echo /lidar_objects
ros2 topic echo /lidar_lines
ros2 topic echo /lidar_ranges
```

---

