# Wsparcie ROS2 dla robotów Unitree

[TOC]

# Wprowadzenie

## Czym jest to repozytorium?
To repozytorium zawiera pakiety ROS2 umożliwiające komunikację i sterowanie robotami Unitree, w tym humanoidalnym robotem **Unitree G1 EDU**. 

## Jak to działa?
Unitree SDK2 implementuje łatwy w użyciu mechanizm komunikacji z robotem oparty na **Cyclonedds**. SDK wspiera roboty Unitree Go2, B2, H1 oraz **G1**. Zobacz: https://github.com/unitreerobotics/unitree_sdk2

DDS (Data Distribution Service) jest również używany w ROS2 jako mechanizm komunikacji. Dlatego warstwa niskopoziomowa robotów Unitree Go2, B2, H1 i G1 jest kompatybilna z ROS2. **Wiadomości ROS2 (msg) mogą być bezpośrednio używane do komunikacji i sterowania robotem Unitree bez konieczności opakowywania interfejsu SDK.**

## Dlaczego to ważne dla studentów?
Ten pakiet pozwala wam na:
- 📡 Bezpośrednią komunikację z robotem używając standardowych narzędzi ROS2
- 🎮 Sterowanie robotem G1 EDU na różnych poziomach abstrakcji
- 📊 Odczyt sensorów (IMU, czujniki siły, enkodery silników)
- 🤖 Tworzenie własnych aplikacji robotycznych w ekosystemie ROS2

# Konfiguracja

## Wymagania systemowe
Przetestowane systemy i dystrybucje ROS2:

|System|Dystrybucja ROS2|
|--|--|
|Ubuntu 20.04|foxy|
|Ubuntu 22.04|humble (zalecana)|

### Środowisko Docker (opcjonalne)
Jeśli chcesz bezpośrednio użyć **środowiska deweloperskiego Docker**, możesz odnieść się do zawartości `Dockerfile` w folderze `.devcontainer`.

Możesz również użyć:
- Funkcji **Dev Container** w VSCode lub innych IDE do utworzenia środowiska deweloperskiego
- **Github Codespace** do szybkiego utworzenia środowiska deweloperskiego online

Jeśli napotkasz problemy z kompilacją, możesz odnieść się do skryptów kompilacji w `.github/workflows/` lub zadać pytania w **issues**.

## Instalacja pakietu Unitree Robot ROS2

Jako przykład używamy ROS2 foxy. Jeśli potrzebujesz innej wersji ROS2, zamień "foxy" na aktualną nazwę wersji ROS2 w odpowiednich miejscach.

### 📥 Instalacja ROS2 foxy
Instalacja ROS2 foxy: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

### 📦 Klonowanie repozytorium
Otwórz terminal (Ctrl+Alt+T) i sklonuj repozytorium:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2
```

**Struktura repozytorium:**
- **cyclonedds_ws**: Przestrzeń robocza pakietów Unitree ROS2. Wiadomości (msg) dla robotów Unitree znajdują się w podfolderach:
  - `cyclonedds_ws/unitree/unitree_go` - dla Go2/B2
  - `cyclonedds_ws/unitree/unitree_api` - interfejs API
  - `cyclonedds_ws/unitree/unitree_hg` - dla H1/G1
- **example**: Przestrzeń robocza z przykładami demonstracyjnymi

## Instalacja zależności i kompilacja

### 1. Instalacja zależności
```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
sudo apt install libyaml-cpp-dev
```

**Co to robi?**
- `rmw-cyclonedds-cpp` - implementacja CycloneDDS dla ROS2 (middleware komunikacyjny)
- `rosidl-generator-dds-idl` - generator definicji wiadomości dla DDS
- `libyaml-cpp-dev` - biblioteka do parsowania plików YAML

### 2. Kompilacja CycloneDDS (można pominąć dla Humble)
**Ważne:** Wersja CycloneDDS używana przez roboty Unitree to **0.10.2**. Aby komunikować się z robotami Unitree używając ROS2, konieczna jest zmiana implementacji DDS. Zobacz: https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html

⚠️ **UWAGA:** Przed kompilacją CycloneDDS upewnij się, że środowisko ROS2 **NIE** zostało załadowane (source) przy uruchamianiu terminala. W przeciwnym razie może to spowodować błędy kompilacji.

**Jeśli dodałeś `source /opt/ros/foxy/setup.bash` do pliku `~/.bashrc`**, musisz go zakomentować:

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
```

Zakomentuj linię (dodaj # na początku):
```bash
# source /opt/ros/foxy/setup.bash 
```

Zamknij i otwórz ponownie terminal, a następnie skompiluj CycloneDDS:

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
# Jeśli kompilacja się nie powiedzie, spróbuj najpierw: `export LD_LIBRARY_PATH=/opt/ros/foxy/lib`
colcon build --packages-select cyclonedds  # Kompiluj pakiet cyclonedds
```

### 3. Kompilacja pakietów unitree_go, unitree_hg i unitree_api
Po skompilowaniu CycloneDDS, do kompilacji pakietów unitree potrzebne są zależności ROS2. Dlatego przed kompilacją trzeba załadować środowisko ROS2:

```bash
source /opt/ros/foxy/setup.bash  # Załaduj środowisko ROS2
colcon build  # Skompiluj wszystkie pakiety w przestrzeni roboczej
```

**Co kompilujemy?**
- Definicje wiadomości dla różnych modeli robotów Unitree
- Interfejsy komunikacyjne zgodne z ROS2
- Przykładowe programy demonstracyjne

## Połączenie z robotem Unitree

### 1. Konfiguracja sieci

**Krok 1:** Połącz robota Unitree i komputer kablem Ethernet.

**Krok 2:** Sprawdź interfejs sieciowy za pomocą `ifconfig`. Na przykład, na poniższym obrazku jest to "enp3s0":

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5d22c143-5dad-4964-81f3-55864906a9f0.png)

**Krok 3:** Otwórz ustawienia sieci, znajdź interfejs sieciowy, do którego podłączony jest robot. W ustawieniach IPv4:
- Zmień tryb IPv4 na **manual** (ręczny)
- Ustaw adres na: **192.168.123.99**
- Ustaw maskę na: **255.255.255.0**

Po zakończeniu kliknij "Zastosuj" i poczekaj na ponowne połączenie sieci.

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/721e1660-04dc-42b7-8d6e-14799afe2165.png)

**Krok 4:** Edytuj plik `setup.sh`:
```bash
sudo gedit ~/unitree_ros2/setup.sh
```

```bash
#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

**Gdzie "enp3s0" to nazwa interfejsu sieciowego, do którego podłączony jest robot Unitree.**
Zmień go na odpowiedni interfejs sieciowy zgodnie z rzeczywistą sytuacją.

**Krok 5:** Załaduj środowisko:
```bash
source ~/unitree_ros2/setup.sh
```

💡 **Wskazówka:** Jeśli nie chcesz ładować skryptu bash za każdym razem, gdy otwierasz nowy terminal, możesz dodać zawartość skryptu do `~/.bashrc`. Jednak należy zachować ostrożność, gdy w systemie współistnieje wiele środowisk ROS.

#### Tryb lokalny (bez robota)
Jeśli komputer nie jest połączony z robotem, ale nadal chcesz używać Unitree ROS2 do symulacji i innych funkcji, możesz użyć lokalnej pętli zwrotnej "lo" jako interfejsu sieciowego:

```bash
source ~/unitree_ros2/setup_local.sh  # Używa "lo" jako interfejsu sieciowego
```

lub

```bash
source ~/unitree_ros2/setup_default.sh  # Bez określonego interfejsu sieciowego
```

### 2. Połączenie i test

Po zakończeniu powyższej konfiguracji zaleca się **ponowne uruchomienie komputera** przed przeprowadzeniem testu.

**Test połączenia:**
Upewnij się, że sieć robota jest poprawnie połączona, otwórz terminal i wpisz:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

Powinieneś zobaczyć listę topików podobną do poniższej:

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5e45e8ec-9248-47eb-8380-798ed0ef468b.png)

**Sprawdź dane z topiku:**
```bash
ros2 topic echo /sportmodestate
```

Zobaczysz dane topiku jak na poniższym obrazku:

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/89214761-6cfb-4b52-bf24-7a5bd9a9806c.png)

### 3. Przykłady

Kod źródłowy przykładów znajduje się w `/example/src/src`.

**Struktura przykładów:**
- `common`: Wspólne funkcje dla wszystkich robotów
- **`g1/lowlevel/g1_low_level_example`**: 🤖 Sterowanie niskopoziomowe dla G1
- **`g1/high_level/`**: Sterowanie wysokopoziomowe dla G1 (ramiona, ruch)
- `h1-2/lowlevel/low_level_ctrl_hg`: Sterowanie niskopoziomowe dla H1-2
- `low_level_ctrl`: Sterowanie niskopoziomowe dla Go2/B2
- `read_low_state`: Odczyt stanu niskopoziomowego z Go2/B2
- **`read_low_state_hg`**: Odczyt stanu niskopoziomowego z G1/H1/H1-2
- `read_motion_state`: Odczyt stanu ruchu (sport mode) z Go2/B2
- **`read_wireless_controller`**: Odczyt stanu kontrolera bezprzewodowego z G1/Go2/B2
- `record_bag`: Przykład nagrywania ROS bag
- `go2/go2_sport_client`: Sterowanie wysokopoziomowe dla Go2
- `go2/go2_stand_example`: Przykładstawania dla Go2
- `go2/go2_robot_state_client`: Przykład stanu robota dla Go2

**Kompilacja przykładów:**
Otwórz terminal i wpisz:

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example
colcon build
```

**Uruchomienie przykładu:**
Po kompilacji uruchom w terminalu:

```bash
./install/unitree_ros2_example/bin/read_motion_state 
```

Zobaczysz informacje o stanie robota wypisywane w terminalu:

```bash
[INFO] [1697525196.266174885] [motion_state_suber]: Position -- x: 0.567083; y: 0.213920; z: 0.052338; body height: 0.320000
[INFO] [1697525196.266230044] [motion_state_suber]: Velocity -- vx: -0.008966; vy: -0.001431; vz: -0.019455; yaw: -0.002131
[INFO] [1697525196.266282725] [motion_state_suber]: Foot position and velcity relative to body -- num: 0; x: 0.204149; y: -0.145194; z: -0.067804, vx: 0.002683; vy: 0.003745; vz: -0.010052
```

# Użycie - Podstawowe funkcje

## 📡 Pobieranie danych ze stanu robota

### 1. Stan trybu sportowego (SportMode State)

**Co to jest?**
Stan trybu sportowego zawiera informacje o pozycji, prędkości, pozycji nóg i innych stanach ruchu robota.

**Jak go uzyskać?**
Subskrybując topik `"lf/sportmodestate"` lub `"sportmodestate"`, gdzie "lf" oznacza niską częstotliwość.

**Struktura wiadomości SportModeState:**
```C++
TimeSpec stamp              // Znacznik czasu
uint32 error_code           // Kod błędu
IMUState imu_state          // Stan IMU (czujnik inercyjny)
uint8 mode                  // Tryb sportowy (patrz poniżej)
float32 progress            // Czy wykonywana jest akcja taneczna? 0-nie, 1-tak
uint8 gait_type             // Typ chodu (patrz poniżej)
float32 foot_raise_height   // Wysokość podniesienia stopy
float32[3] position         // Pozycja robota [x, y, z]
float32 body_height         // Wysokość korpusu
float32[3] velocity         // Prędkość robota [vx, vy, vz]
float32 yaw_speed           // Prędkość obrotu (yaw)
float32[4] range_obstacle   // Odległość do przeszkód
int16[4] foot_force         // Siły nóg
float32[12] foot_position_body  // Pozycje stóp we współrzędnych korpusu
float32[12] foot_speed_body     // Prędkości stóp we współrzędnych korpusu
```

**Tryby sportowe (mode):**
- 0: **idle** - bezczynność, domyślne stanie
- 1: **balanceStand** - stanie z balansowaniem
- 2: **pose** - poza
- 3: **locomotion** - ruch/chodzenie
- 4: reserve (zarezerwowane)
- 5: **lieDown** - leżenie
- 6: **jointLock** - blokada stawów
- 7: **damping** - tłumienie
- 8: **recoveryStand** - powrót do stania
- 9: reserve (zarezerwowane)
- 10: **sit** - siedzenie
- 11: **frontFlip** - salto w przód
- 12: **frontJump** - skok w przód
- 13: **frontPounce** - rzut w przód

**Typy chodu (gait_type):**
- 0: **idle** - bezczynność
- 1: **trot** - kłus
- 2: **run** - bieg
- 3: **climb stair** - wchodzenie po schodach
- 4: **forwardDownStair** - schodzenie po schodach
- 9: **adjust** - dostosowanie

Więcej szczegółów: https://support.unitree.com/home/en/developer/sports_services

**Pełny przykład:** `/example/src/read_motion_state.cpp`

**Uruchomienie:**
```bash
./install/unitree_ros2_example/bin/read_motion_state 
```

### 2. Stan niskopoziomowy (Low-Level State)

**Co to jest?**
Stan niskopoziomowy zawiera stany silników, informacje o zasilaniu i inne stany niskiego poziomu.

**Jak go uzyskać?**
Subskrybując topik `"lf/lowstate"` lub `"lowstate"`.

**Struktura wiadomości LowState:**
```C++
uint8[2] head               // Nagłówek
uint8 level_flag            // Flaga poziomu
uint8 frame_reserve         // Rezerwa ramki
uint32[2] sn                // Numer seryjny
uint32[2] version           // Wersja
uint16 bandwidth            // Szerokość pasma
IMUState imu_state          // Stan IMU
MotorState[20] motor_state  // Stan silników (tablica 20 silników)
BmsState bms_state          // Stan systemu zarządzania baterią
int16[4] foot_force         // Siły nóg
int16[4] foot_force_est     // Oszacowane siły nóg
uint32 tick                 // Licznik taktów
uint8[40] wireless_remote   // Dane kontrolera bezprzewodowego
uint8 bit_flag              // Flaga bitowa
float32 adc_reel            // ADC
int8 temperature_ntc1       // Temperatura NTC1
int8 temperature_ntc2       // Temperatura NTC2
float32 power_v             // Napięcie zasilania [V]
float32 power_a             // Prąd zasilania [A]
uint16[4] fan_frequency     // Częstotliwości wentylatorów
uint32 reserve              // Rezerwa
uint32 crc                  // Suma kontrolna
```

**Struktura MotorState:**
```C++
uint8 mode          // Tryb, 0x01 dla sterowania
float32 q           // Kąt stawu [rad]
float32 dq          // Prędkość kątowa [rad/s]
float32 ddq         // Przyspieszenie kątowe [rad/s²]
float32 tau_est     // Oszacowany moment obrotowy [N⋅m]
float32 q_raw       // Surowe dane q
float32 dq_raw      // Surowe dane dq
float32 ddq_raw     // Surowe dane ddq
int8 temperature    // Temperatura silnika [°C]
uint32 lost         // Licznik utraconych pakietów
uint32[2] reserve   // Rezerwa
```

Więcej szczegółów: https://support.unitree.com/home/en/developer/Basic_services

**Pełny przykład:** `example/src/read_low_state.cpp`

### 3. Kontroler bezprzewodowy (Wireless Controller)

**Co to jest?**
Stan kontrolera bezprzewodowego można uzyskać subskrybując topik `"/wirelesscontroller"`.

**Struktura wiadomości WirelessController:**
```C++
float32 lx      // Lewa gałka joysticka - oś X, zakres [-1.0~1.0]
float32 ly      // Lewa gałka joysticka - oś Y, zakres [-1.0~1.0]
float32 rx      // Prawa gałka joysticka - oś X, zakres [-1.0~1.0]
float32 ry      // Prawa gałka joysticka - oś Y, zakres [-1.0~1.0]
uint16 keys     // Wartości przycisków
```

Więcej szczegółów: https://support.unitree.com/home/en/developer/Get_remote_control_status

**Pełny przykład:** `example/src/read_wireless_controller.cpp`

## 🎮 Sterowanie robotem

### 1. Tryb sportowy (SportMode Control)

**Jak to działa?**
Sterowanie trybem sportowym jest realizowane przez mechanizm **request/response** (żądanie/odpowiedź).

**Jak sterować?**
Wysyłając wiadomość `unitree_api::msg::Request` do topiku `"/api/sport/request"`.

**Przykład sterowania postawą robota:**

```C++
// Utwórz wydawcę ROS2
rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber = 
    this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

SportClient sport_req;              // Klient trybu sportowego
unitree_api::msg::Request req;      // Wiadomość żądania
sport_req.Euler(req, roll, pitch, yaw);  // Uzyskaj żądanie zmiany postawy

req_puber->publish(req);            // Opublikuj żądanie
```

**Klasa SportClient:**
Klasa `SportClient` (`/example/src/common/ros2_sport_client.cpp`) dostarcza gotowe metody do tworzenia różnych żądań sterowania.

Więcej szczegółów: https://support.unitree.com/home/en/developer/sports_services

**Pełny przykład:** `example/src/sport_mode_ctrl.cpp`

**Uruchomienie:**
```bash
./install/unitree_ros2_example/bin/sport_mode_ctrl
```
Po 1 sekundzie od uruchomienia programu robot będzie chodził tam i z powrotem w kierunku X.

### 2. Sterowanie silnikami (Motor Control)

**Co można kontrolować?**
- Moment obrotowy (torque)
- Pozycję (position)
- Prędkość (velocity)

**Jak to działa?**
Publikując wiadomość `unitree_go::msg::LowCmd` do topiku `"/lowcmd"`.

**Struktura wiadomości LowCmd:**
```C++
uint8[2] head               // Nagłówek
uint8 level_flag            // Flaga poziomu
uint8 frame_reserve         // Rezerwa ramki
uint32[2] sn                // Numer seryjny
uint32[2] version           // Wersja
uint16 bandwidth            // Szerokość pasma
MotorCmd[20] motor_cmd      // Komendy silników
BmsCmd bms_cmd              // Komenda BMS
uint8[40] wireless_remote   // Kontroler bezprzewodowy
uint8[12] led               // LEDy
uint8[2] fan                // Wentylatory
uint8 gpio                  // GPIO
uint32 reserve              // Rezerwa
uint32 crc                  // Suma kontrolna
```

**Struktura MotorCmd:**
```C++
uint8 mode;     // Tryb (Tryb FOC -> 0x01, Tryb stop -> 0x00)
float q;        // Docelowa pozycja [rad]
float dq;       // Docelowa prędkość [rad/s]
float tau;      // Docelowy moment [N⋅m]
float kp;       // Wzmocnienie proporcjonalne regulatora PD
float kd;       // Wzmocnienie różniczkowe regulatora PD
unsigned long reserve[3];  // Rezerwa
```

**⚠️ Uwaga o bezpieczeństwie:**
Sterowanie niskopoziomowe wymaga ostrożności! Nieprawidłowe wartości mogą spowodować niestabilne zachowanie robota.

Więcej szczegółów: https://support.unitree.com/home/en/developer/Basic_services

**Pełny przykład:** `example/src/low_level_ctrl.cpp`

**Uruchomienie:**
```bash
./install/unitree_ros2_example/bin/low_level_ctrl
```
Silnik biodra i silnik łydki prawej tylnej nogi obracają się do odpowiedniego kąta stawu.

## 📊 Wizualizacja w RViz

Możemy również użyć RViz do wizualizacji danych robota Unitree. Poniżej przykład wizualizacji danych z lidaru:

**Krok 1:** Wyświetl wszystkie topiki:
```bash
ros2 topic list
```

![image](docs/image/piFtteJ.png)

**Krok 2:** Znajdź topik lidaru:
```bash
utlidar/cloud
```

**Krok 3:** Sprawdź frame_id lidaru:
```bash
ros2 topic echo --no-arr /utlidar/cloud
```
gdzie frame_id: **utlidar_lidar**

![image](docs/image/piFtdF1.png)

**Krok 4:** Uruchom RViz:
```bash
ros2 run rviz2 rviz2
```

**Krok 5:** W RViz2:
- Dodaj topik Pointcloud: `utlidar/cloud`
- Zmień Fixed Frame na `utlidar_lidar`

Dane z lidaru będą wyświetlane w RViz2:

![image](docs/image/piFtsyD.png)
![image](docs/image/piFtyOe.png)

---

## 📚 Dalsze materiały edukacyjne

Aby głębiej zrozumieć pracę z robotem Unitree G1 EDU, przeczytaj:
- **[Przewodnik dla studentów](docs/PRZEWODNIK_STUDENTA.md)** - szczegółowy przewodnik po całym ekosystemie
- **[Praktyczny przewodnik G1 EDU](docs/G1_EDU_PRAKTYCZNY_PRZEWODNIK.md)** - specyfika pracy z robotem humanoidalnym G1
- **[Przykłady projektów](docs/PRZYKLADY_PROJEKTOW_G1.md)** - inspiracje do własnych projektów

## 🆘 Pomoc i wsparcie

- Dokumentacja oficjalna Unitree: https://support.unitree.com/
- Issues w tym repozytorium: https://github.com/unitreerobotics/unitree_ros2/issues
- ROS2 Documentation: https://docs.ros.org/

---

**Powodzenia w nauce robotyki! 🤖🎓**
