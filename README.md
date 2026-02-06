# Q-Learning ve RRT ile Labirent Gezgin Robot

Bu proje, ROS 2 ve Gazebo kullanarak labirent ortamında otonom navigasyon yapan bir robot simülasyonudur. Proje iki ana yaklaşım içerir:
1.  **Q-Learning (Pekiştirmeli Öğrenme):** Robotun labirentte kendi kendine öğrenerek yolu bulması.
2.  **RRT (Rapidly-exploring Random Tree):** Algoritmik olarak en kısa yolun planlanması ve izlenmesi.

## Özellikler

- **Gazebo Simülasyonu:** Özel labirent tasarımı ve TurtleBot3 robot modeli.
- **Q-Learning Ajanı:** Durum makinesi (State Machine) tabanlı, çarpışma önleyici ve asenkron reset özellikli öğrenme algoritması.
- **RRT Planlayıcı:** Hızlı yol bulma, engellerden kaçınma ve düzgünleştirilmiş yol takibi.
- **Veri Görselleştirme:** Eğitim ve planlama sonuçlarını (başarı oranı, adım sayısı, süre vb.) grafiklerle gösterme.

## Kurulum

Bu paket **ROS 2 Humble** (veya Foxy) ve **Gazebo** gerektirir. TurtleBot3 paketlerinin kurulu olduğundan emin olun:
`sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs`

1.  Yeni bir çalışma alanı oluşturun (veya mevcut olanı kullanın):
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  Projeyi klonlayın:
    ```bash
    git clone https://github.com/KULLANICI_ADI/rl_maze_nav.git
    ```

3.  Bağımlılıkları yükleyin ve derleyin:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash
    ```

4.  TurtleBot3 modelini ayarlayın:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ```

## Kullanım

### 1. RRT Planlayıcı (Yol Planlama)
Robotun RRT algoritması ile oluşturulan yolu izlemesi için:

```bash
ros2 launch rl_maze_nav maze_rrt.launch.py
```
*Not: Simülasyon bittiğinde veya Ctrl+C ile durdurulduğunda performans grafikleri otomatik olarak açılır.*

### 2. Q-Learning (Eğitim)
Robotun deneme-yanılma ile labirenti öğrenmesi için:

```bash
ros2 launch rl_maze_nav maze_dqn.launch.py
```
*Bu modda robot başlangıçta çok fazla hata yapabilir, zamanla öğrenerek hedefe ulaşmayı başaracaktır.*

## Dosya Yapısı

- `rl_maze_nav/`: Kaynak kodlar (Planlayıcı ve Öğrenme algoritmaları)
- `launch/`: Başlatma dosyaları
- `worlds/`: Gazebo labirent dünyası
- `resource/`: Paket kaynak dosyaları
