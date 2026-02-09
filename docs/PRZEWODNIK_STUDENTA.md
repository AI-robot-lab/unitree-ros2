# Przewodnik Studenta - Unitree ROS2

## 📖 Spis treści
1. [Wprowadzenie](#wprowadzenie)
2. [Podstawy komunikacji z robotem](#podstawy-komunikacji-z-robotem)
3. [Architektura systemu](#architektura-systemu)
4. [Poziomy sterowania](#poziomy-sterowania)
5. [Pierwszy program](#pierwszy-program)
6. [Zaawansowane przykłady](#zaawansowane-przykłady)
7. [Najczęstsze problemy](#najczęstsze-problemy)

---

## Wprowadzenie

### Czym jest ROS2?
**ROS2 (Robot Operating System 2)** to framework do tworzenia aplikacji robotycznych. To **nie jest** system operacyjny w tradycyjnym znaczeniu, ale zestaw bibliotek i narzędzi, które ułatwiają tworzenie złożonych systemów robotycznych.

### Dlaczego używamy ROS2 z robotami Unitree?
- 🔄 **Modularność** - każda funkcja robota może być osobnym programem (nodem)
- 📡 **Komunikacja** - łatwa wymiana danych między programami poprzez topiki
- 🛠️ **Narzędzia** - gotowe narzędzia do debugowania, wizualizacji i nagrywania danych
- 🌍 **Społeczność** - ogromna społeczność robotyków używających ROS2

### Jak komunikuje się robot Unitree?
Robot Unitree używa **DDS (Data Distribution Service)** jako warstwy komunikacyjnej. ROS2 również używa DDS, co oznacza, że mogą bezpośrednio ze sobą rozmawiać!

```
┌─────────────┐         DDS          ┌─────────────┐
│  Twój kod   │ ←──────────────────→ │   Robot     │
│  (ROS2)     │    (CycloneDDS)      │  Unitree    │
└─────────────┘                      └─────────────┘
```

---

## Podstawy komunikacji z robotem

### Koncepcja Topików (Topics)

**Topik** to nazwany kanał komunikacyjny, przez który przesyłane są wiadomości.

**Analogia:** Pomyśl o topiku jak o kanale telewizyjnym:
- Robot "nadaje" swoje dane na różnych kanałach (topikach)
- Twój program "subskrybuje" (ogląda) interesujące cię kanały

```
Robot publikuje dane:                    Twój program subskrybuje:
┌─────────────────┐                     ┌──────────────────┐
│   /lowstate     │ ───────────────────→│  read_low_state  │
│ (stan silników) │                     │   (odczyt)       │
└─────────────────┘                     └──────────────────┘
```

### Typy wiadomości (Messages)

Każdy topik ma określony **typ wiadomości** - strukturę danych, która jest przesyłana.

**Przykład:** `unitree_go::msg::LowState` zawiera:
- Stan wszystkich silników
- Dane z IMU (czujnika inercyjnego)
- Informacje o baterii
- Siły w stopach

### Podstawowe komendy ROS2

```bash
# Wyświetl wszystkie dostępne topiki
ros2 topic list

# Zobacz dane z konkretnego topiku
ros2 topic echo /lowstate

# Sprawdź typ wiadomości topiku
ros2 topic info /lowstate

# Sprawdź częstotliwość publikacji
ros2 topic hz /lowstate

# Zobacz strukturę typu wiadomości
ros2 interface show unitree_go/msg/LowState
```

---

## Architektura systemu

### Poziomy abstrakcji w robotyce Unitree

Robot Unitree działa na kilku poziomach abstrakcji:

```
┌─────────────────────────────────────────┐
│  POZIOM 4: Zadania wysokiego poziomu    │
│  (idź do celu, unikaj przeszkód)        │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│  POZIOM 3: Sport Mode API               │
│  (chód, stanie, siedzenie)              │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│  POZIOM 2: Low-level control            │
│  (bezpośrednie sterowanie silnikami)    │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│  POZIOM 1: Hardware                     │
│  (silniki, czujniki, IMU)               │
└─────────────────────────────────────────┘
```

### Główne topiki robota

#### Dla Go2/B2:
| Topik | Typ | Częstotliwość | Opis |
|-------|-----|---------------|------|
| `/lowstate` | `LowState` | 500 Hz | Stan silników, IMU, baterii |
| `/lf/lowstate` | `LowState` | 20 Hz | To samo, niska częstotliwość |
| `/sportmodestate` | `SportModeState` | 500 Hz | Stan ruchu, pozycja, prędkość |
| `/lf/sportmodestate` | `SportModeState` | 20 Hz | To samo, niska częstotliwość |
| `/lowcmd` | `LowCmd` | - | Komendy dla silników (publikujesz tu) |
| `/api/sport/request` | `Request` | - | Żądania trybu sportowego |

#### Dla G1/H1:
| Topik | Typ | Częstotliwość | Opis |
|-------|-----|---------------|------|
| `/lowstate` | `unitree_hg::msg::LowState` | 500 Hz | Stan 29 silników G1 |
| `/lowcmd` | `unitree_hg::msg::LowCmd` | - | Komendy dla 29 silników |
| `/wirelesscontroller` | `WirelessController` | 50 Hz | Stan kontrolera |

**💡 Wskazówka:** Używaj wersji `/lf/*` (low frequency) gdy nie potrzebujesz wysokiej częstotliwości - oszczędzisz zasoby!

---

## Poziomy sterowania

### 1. Obserwator (Observer) - Poziom podstawowy

**Cel:** Tylko odczyt danych z robota, bez wysyłania komend.

**Użycie:** Monitorowanie, diagnostyka, nauka struktury danych.

**Przykład:** Program `read_low_state` - tylko odbiera i wyświetla stan robota.

```cpp
// Prosty schemat obserwatora
class Observer : public rclcpp::Node {
    rclcpp::Subscription<LowState>::SharedPtr subscriber_;
    
    void callback(const LowState::SharedPtr msg) {
        // Tylko odczytujemy dane
        RCLCPP_INFO(this->get_logger(), "Motor 0: %f", msg->motor_state[0].q);
    }
};
```

### 2. Sport Mode Control - Poziom średnio-zaawansowany

**Cel:** Sterowanie wysokopoziomowe - polecenia typu "idź", "stań", "usiądź".

**Użycie:** Bezpieczne sterowanie, gotowe zachowania robota.

**Bezpieczeństwo:** ✅ Wysoki - robot sam dba o stabilność.

**Przykład:** Wysłanie polecenia zmiany postawy.

```cpp
// Schemat sterowania Sport Mode
SportClient sport_client;
unitree_api::msg::Request request;

// Robot sam oblicza jak bezpiecznie wykonać ruch
sport_client.Move(request, vx, vy, vyaw);  
publisher->publish(request);
```

### 3. Low-Level Control - Poziom zaawansowany

**Cel:** Bezpośrednie sterowanie silnikami - pełna kontrola.

**Użycie:** Zaawansowane algorytmy sterowania, badania.

**Bezpieczeństwo:** ⚠️ Niski - pełna odpowiedzialność za stabilność!

**Przykład:** Bezpośrednie ustawienie pozycji stawu.

```cpp
// Schemat sterowania niskopoziomowego
unitree_hg::msg::LowCmd cmd;

// Bezpośrednio ustawiamy parametry silnika
cmd.motor_cmd[LEFT_HIP_PITCH].mode = 1;     // Włącz silnik
cmd.motor_cmd[LEFT_HIP_PITCH].q = 0.5;      // Docelowy kąt [rad]
cmd.motor_cmd[LEFT_HIP_PITCH].dq = 0.0;     // Docelowa prędkość
cmd.motor_cmd[LEFT_HIP_PITCH].kp = 100.0;   // Wzmocnienie P
cmd.motor_cmd[LEFT_HIP_PITCH].kd = 1.0;     // Wzmocnienie D
cmd.motor_cmd[LEFT_HIP_PITCH].tau = 0.0;    // Dodatkowy moment

publisher->publish(cmd);
```

**⚠️ UWAGA:** Nieprawidłowe wartości w trybie niskopoziomowym mogą spowodować:
- Gwałtowne ruchy robota
- Uszkodzenie mechaniczne
- Upadek robota

**Zasady bezpieczeństwa:**
1. Zawsze testuj z małymi wartościami `kp` i `kd`
2. Stopniowo zwiększaj wartości docelowe
3. Miej zawsze wyłącznik awaryjny (kontroler)
4. Nigdy nie testuj w pobliżu ludzi lub cennych przedmiotów

---

## Pierwszy program

### Program 1: Odczyt stanu IMU

**Cel:** Nauczyć się odbierać i interpretować dane z czujnika inercyjnego.

**Co robi IMU?**
- Mierzy orientację robota (kąty Roll, Pitch, Yaw)
- Mierzy prędkość kątową (żyroskop)
- Mierzy przyspieszenie liniowe (akcelerometr)

**Kod z komentarzami:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"

class IMUReader : public rclcpp::Node {
public:
    IMUReader() : Node("imu_reader") {
        // Tworzymy subskrybenta dla topiku lowstate
        // Parametr "10" to rozmiar kolejki wiadomości
        subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate",  // Nazwa topiku
            10,          // Rozmiar kolejki
            [this](const unitree_go::msg::LowState::SharedPtr msg) {
                this->imu_callback(msg);
            }
        );
    }

private:
    void imu_callback(const unitree_go::msg::LowState::SharedPtr msg) {
        // Wyciągamy dane IMU z wiadomości
        auto imu = msg->imu_state;
        
        // Kąty Eulera (w radianach)
        // Roll: obrót wokół osi X (przechylenie na boki)
        // Pitch: obrót wokół osi Y (przechylenie przód-tył)
        // Yaw: obrót wokół osi Z (obrót w poziomie)
        double roll = imu.rpy[0];
        double pitch = imu.rpy[1];
        double yaw = imu.rpy[2];
        
        // Konwersja z radianów na stopnie dla lepszej czytelności
        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double yaw_deg = yaw * 180.0 / M_PI;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Orientacja: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°",
                    roll_deg, pitch_deg, yaw_deg);
        
        // Prędkości kątowe (żyroskop)
        RCLCPP_INFO(this->get_logger(),
                    "Żyroskop: wx=%.2f wy=%.2f wz=%.2f rad/s",
                    imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
    }
    
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;
};

int main(int argc, char** argv) {
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    
    // Uruchomienie noda
    rclcpp::spin(std::make_shared<IMUReader>());
    
    // Zamknięcie ROS2
    rclcpp::shutdown();
    return 0;
}
```

**Jak zbudować i uruchomić:**

1. Dodaj do `CMakeLists.txt`:
```cmake
add_executable(imu_reader src/imu_reader.cpp)
ament_target_dependencies(imu_reader rclcpp unitree_go)
install(TARGETS imu_reader DESTINATION lib/${PROJECT_NAME})
```

2. Zbuduj:
```bash
cd ~/unitree_ros2/example
colcon build --packages-select unitree_ros2_example
source install/setup.bash
```

3. Uruchom:
```bash
./install/unitree_ros2_example/bin/imu_reader
```

**Co powinieneś zobaczyć:**
```
[INFO] [imu_reader]: Orientacja: Roll=0.5° Pitch=-1.2° Yaw=45.3°
[INFO] [imu_reader]: Żyroskop: wx=0.01 wy=-0.02 wz=0.00 rad/s
```

---

### Program 2: Monitorowanie temperatury silników

**Cel:** Nauka iteracji po tablicach stanów i warunkowego wyświetlania.

**Dlaczego to ważne?**
Przegrzanie silników może prowadzić do:
- Ograniczenia mocy
- Uszkodzenia silnika
- Wyłączenia awaryjnego robota

**Kod:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"

class MotorTemperatureMonitor : public rclcpp::Node {
public:
    MotorTemperatureMonitor() : Node("motor_temp_monitor") {
        subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lf/lowstate",  // Używamy niskiej częstotliwości - wystarczy
            10,
            [this](const unitree_go::msg::LowState::SharedPtr msg) {
                this->check_temperatures(msg);
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Monitor temperatury silników uruchomiony");
    }

private:
    void check_temperatures(const unitree_go::msg::LowState::SharedPtr msg) {
        // Dla Go2/B2 mamy 12 silników (0-11)
        const int NUM_MOTORS = 12;
        const int TEMP_WARNING = 60;  // °C - ostrzeżenie
        const int TEMP_CRITICAL = 70; // °C - krytyczna
        
        bool any_warning = false;
        
        // Iterujemy po wszystkich silnikach
        for (int i = 0; i < NUM_MOTORS; i++) {
            int temp = msg->motor_state[i].temperature;
            
            // Sprawdzamy poziomy temperatur
            if (temp >= TEMP_CRITICAL) {
                RCLCPP_ERROR(this->get_logger(),
                            "⚠️ KRYTYCZNIE! Silnik %d: %d°C", i, temp);
                any_warning = true;
            }
            else if (temp >= TEMP_WARNING) {
                RCLCPP_WARN(this->get_logger(),
                           "⚠️ UWAGA! Silnik %d: %d°C", i, temp);
                any_warning = true;
            }
        }
        
        // Jeśli wszystko OK, wyświetl podsumowanie co 5 sekund
        if (!any_warning && (msg->tick % 100 == 0)) {  // ~5s przy 20Hz
            int max_temp = find_max_temperature(msg);
            RCLCPP_INFO(this->get_logger(),
                       "✓ Wszystkie silniki OK (max temp: %d°C)", max_temp);
        }
    }
    
    int find_max_temperature(const unitree_go::msg::LowState::SharedPtr msg) {
        int max_temp = 0;
        for (int i = 0; i < 12; i++) {
            int temp = msg->motor_state[i].temperature;
            if (temp > max_temp) max_temp = temp;
        }
        return max_temp;
    }
    
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTemperatureMonitor>());
    rclcpp::shutdown();
    return 0;
}
```

**Pojęcia wyjaśnione:**
- `RCLCPP_INFO` - normalna informacja (zielony tekst)
- `RCLCPP_WARN` - ostrzeżenie (żółty tekst)
- `RCLCPP_ERROR` - błąd (czerwony tekst)
- `msg->tick` - licznik wiadomości, przydatny do timingu

---

## Zaawansowane przykłady

### Program 3: Prosty joystick do kontroli

**Cel:** Odczyt kontrolera bezprzewodowego i sterowanie prędkością robota.

**Schemat działania:**
```
Kontroler → [Odczyt] → [Mapowanie] → [Komenda ruchu] → Robot
```

**Kod uproszczony (pseudo-kod z wyjaśnieniami):**

```cpp
class JoystickController : public rclcpp::Node {
private:
    // Subskrybent kontrolera
    rclcpp::Subscription<WirelessController>::SharedPtr joy_sub_;
    
    // Wydawca komend ruchu
    rclcpp::Publisher<Request>::SharedPtr cmd_pub_;
    
    SportClient sport_client_;  // Klient do tworzenia komend

    void joystick_callback(const WirelessController::SharedPtr joy) {
        // KROK 1: Odczytaj wartości joysticków
        // Lewy joystick: ruch przód/tył i lewo/prawo
        float ly = joy->ly;  // Przód(+)/tył(-), zakres [-1, 1]
        float lx = joy->lx;  // Lewo(+)/prawo(-), zakres [-1, 1]
        
        // Prawy joystick: obrót
        float rx = joy->rx;  // Obrót lewo/prawo, zakres [-1, 1]
        
        // KROK 2: Mapuj wartości joysticka na prędkości robota
        // Maksymalne prędkości (bezpieczne wartości)
        const float MAX_VX = 0.5;    // m/s przód/tył
        const float MAX_VY = 0.3;    // m/s lewo/prawo
        const float MAX_VYAW = 0.5;  // rad/s obrót
        
        float vx = ly * MAX_VX;      // Prędkość do przodu
        float vy = lx * MAX_VY;      // Prędkość na boki
        float vyaw = rx * MAX_VYAW;  // Prędkość obrotu
        
        // KROK 3: Strefa martwa (dead zone)
        // Joysticki nigdy nie są idealnie w centrum (0.0)
        // Ignorujemy małe wartości aby uniknąć driftu
        const float DEADZONE = 0.1;
        
        if (std::abs(vx) < DEADZONE) vx = 0.0;
        if (std::abs(vy) < DEADZONE) vy = 0.0;
        if (std::abs(vyaw) < DEADZONE) vyaw = 0.0;
        
        // KROK 4: Utwórz i wyślij komendę ruchu
        unitree_api::msg::Request request;
        sport_client_.Move(request, vx, vy, vyaw);
        cmd_pub_->publish(request);
        
        // KROK 5: Informuj użytkownika (opcjonalnie)
        if (vx != 0.0 || vy != 0.0 || vyaw != 0.0) {
            RCLCPP_INFO(this->get_logger(),
                       "Ruch: vx=%.2f vy=%.2f vyaw=%.2f", vx, vy, vyaw);
        }
    }
};
```

**Wyjaśnienie koncepcji:**

1. **Mapowanie wartości:** Joystick daje wartości [-1, 1], ale robot oczekuje prędkości w m/s lub rad/s. Musimy przeskalować.

2. **Strefa martwa (deadzone):** Joysticki analogowe rzadko pokazują dokładnie 0.0 gdy są w centrum. Małe wartości (np. < 0.1) traktujemy jako zero.

3. **Bezpieczne limity:** Nie używaj maksymalnych prędkości robota od razu. Zacznij od małych wartości!

---

### Program 4: Nagrywanie trajektorii

**Cel:** Zapamiętaj pozycje stawów w trybie manualnym, potem odtwórz.

**Zastosowanie:** Nauka przez demonstrację (learning from demonstration).

**Schemat:**
```
[Tryb nagrywania] → Zapisz pozycje → [Tryb odtwarzania] → Wykonaj ruch
```

**Uproszczona implementacja:**

```cpp
class TrajectoryRecorder : public rclcpp::Node {
private:
    // Stan aplikacji
    enum State { IDLE, RECORDING, PLAYING };
    State current_state_ = IDLE;
    
    // Zapisane pozycje
    std::vector<std::array<float, 12>> recorded_positions_;
    
    // Timer do kontroli częstotliwości
    rclcpp::TimerBase::SharedPtr timer_;
    
    void state_machine() {
        switch(current_state_) {
            case IDLE:
                // Czekaj na komendę użytkownika
                break;
                
            case RECORDING:
                // Odczytuj aktualne pozycje i zapisuj
                record_current_position();
                break;
                
            case PLAYING:
                // Odtwarzaj zapisane pozycje
                play_next_position();
                break;
        }
    }
    
    void record_current_position() {
        // Pobierz aktualny stan (z ostatniej wiadomości)
        std::array<float, 12> positions;
        for (int i = 0; i < 12; i++) {
            positions[i] = last_state_.motor_state[i].q;
        }
        
        // Dodaj do trajektorii
        recorded_positions_.push_back(positions);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Zapisano punkt %zu", recorded_positions_.size());
    }
    
    void play_next_position() {
        // Wyślij kolejną pozycję do silników
        // ... implementacja sterowania ...
    }
};
```

**Rozszerzenia dla zainteresowanych:**
- Zapisz trajektorię do pliku (JSON, YAML)
- Dodaj interpolację między punktami (gładki ruch)
- Dodaj kontrolę prędkości odtwarzania

---

## Najczęstsze problemy

### Problem 1: "ros2 topic list" nie pokazuje topików robota

**Możliwe przyczyny:**
1. ❌ Nie załadowano środowiska `source ~/unitree_ros2/setup.sh`
2. ❌ Robot nie jest podłączony lub nieprawidłowa konfiguracja sieci
3. ❌ Zły interfejs sieciowy w `setup.sh`

**Rozwiązanie krok po kroku:**

```bash
# 1. Sprawdź interfejs sieciowy
ifconfig
# Znajdź interfejs z adresem 192.168.123.xxx

# 2. Edytuj setup.sh z prawidłowym interfejsem
gedit ~/unitree_ros2/setup.sh
# Zmień "enp3s0" na twój interfejs

# 3. Załaduj środowisko
source ~/unitree_ros2/setup.sh

# 4. Sprawdź ponownie
ros2 topic list
```

### Problem 2: Kompilacja się nie udaje

**Błąd:** `Could not find a package configuration file provided by "unitree_go"`

**Rozwiązanie:**
```bash
# 1. Sprawdź czy sklonowałeś cyclonedds_ws
ls ~/unitree_ros2/cyclonedds_ws/src/

# 2. Upewnij się, że cyclonedds_ws jest zbudowany
cd ~/unitree_ros2/cyclonedds_ws
source /opt/ros/foxy/setup.bash
colcon build

# 3. Załaduj środowisko cyclonedds_ws
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash

# 4. Teraz kompiluj swój projekt
cd ~/unitree_ros2/example
colcon build
```

### Problem 3: Robot nie reaguje na komendy

**Możliwe przyczyny:**
1. ❌ Robot nie jest w odpowiednim trybie
2. ❌ Komenda ma nieprawidłową sumę kontrolną (CRC)
3. ❌ Niewłaściwa częstotliwość wysyłania

**Dla sterowania niskopoziomowego:**
```cpp
// ZAWSZE obliczaj CRC przed wysłaniem!
#include "common/motor_crc.h"  // dla Go2/B2
// lub
#include "common/motor_crc_hg.h"  // dla G1/H1

unitree_go::msg::LowCmd cmd;
// ... wypełnij cmd ...
get_crc(cmd);  // WAŻNE! Oblicz sumę kontrolną
publisher->publish(cmd);
```

**Częstotliwość:** Komendy niskopoziomowe powinny być wysyłane z częstotliwością **500 Hz** (co 2ms).

### Problem 4: Wartości z czujników wyglądają na błędne

**Przykład:** IMU pokazuje dziwne wartości.

**Sprawdź jednostki:**
- Kąty w IMU są w **radianach**, nie stopniach!
- Pozycje stawów w **radianach**
- Prędkości kątowe w **rad/s**
- Momenty obrotowe w **N⋅m**

**Konwersje:**
```cpp
// Radiany → Stopnie
double degrees = radians * 180.0 / M_PI;

// Stopnie → Radiany  
double radians = degrees * M_PI / 180.0;
```

### Problem 5: Robot zachowuje się niestabilnie

**Gdy używasz sterowania niskopoziomowego:**

⚠️ **STOP! Sprawdź:**
1. Czy wartości `kp` i `kd` nie są za duże?
   - Zacznij od małych: `kp=10`, `kd=0.5`
   - Stopniowo zwiększaj
2. Czy wysyłasz komendy regularnie (500 Hz)?
   - Użyj `rclcpp::TimerBase` z `std::chrono::milliseconds(2)`
3. Czy obliczasz CRC?
   - Zawsze wywołaj `get_crc(cmd)` przed publikacją!

**Bezpieczne wartości startowe dla G1:**
```cpp
// Bezpieczne dla nóg
low_cmd.motor_cmd[i].kp = 100.0;
low_cmd.motor_cmd[i].kd = 1.0;

// Bezpieczne dla ramion  
low_cmd.motor_cmd[i].kp = 50.0;
low_cmd.motor_cmd[i].kd = 1.0;
```

---

## Narzędzia diagnostyczne

### 1. rqt_graph - Wizualizacja grafów

```bash
ros2 run rqt_graph rqt_graph
```

Pokazuje:
- Jakie nody działają
- Jakie topiki istnieją
- Kto publikuje/subskrybuje

### 2. ros2 topic hz - Sprawdzanie częstotliwości

```bash
ros2 topic hz /lowstate
```

Pokazuje ile wiadomości na sekundę jest publikowanych.

### 3. ros2 bag - Nagrywanie danych

```bash
# Nagrywaj wszystkie topiki
ros2 bag record -a

# Nagrywaj wybrane topiki
ros2 bag record /lowstate /sportmodestate

# Odtwarzaj
ros2 bag play nazwa_pliku
```

**Zastosowanie:** Nagrywaj dane podczas testów, analizuj później offline.

---

## Podsumowanie

**Czego się nauczyłeś:**
- ✅ Podstawy komunikacji ROS2 (topiki, wiadomości)
- ✅ Architektura systemu Unitree
- ✅ Różne poziomy sterowania robotem
- ✅ Pisanie prostych i zaawansowanych programów
- ✅ Rozwiązywanie typowych problemów

**Następne kroki:**
1. 📖 Przeczytaj [Praktyczny przewodnik G1 EDU](G1_EDU_PRAKTYCZNY_PRZEWODNIK.md)
2. 💡 Zobacz [Przykłady projektów](PRZYKLADY_PROJEKTOW_G1.md)
3. 🔬 Eksperymentuj z przykładami w folderze `example/`
4. 🤝 Dziel się swoimi doświadczeniami z innymi studentami!

**Pamiętaj:**
- 🔐 Bezpieczeństwo przede wszystkim
- 📚 Czytaj dokumentację
- 🧪 Testuj małe zmiany
- 💾 Zapisuj kod w systemie kontroli wersji (git)
- ❓ Zadawaj pytania w issues

---

**Powodzenia w swoich projektach robotycznych! 🤖🎓**
