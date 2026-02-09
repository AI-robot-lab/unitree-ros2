# Praktyczny Przewodnik: Robot Humanoidalny Unitree G1 EDU

## 📖 Spis treści
1. [Wprowadzenie do G1 EDU](#wprowadzenie-do-g1-edu)
2. [Specyfikacja robota](#specyfikacja-robota)
3. [Kinematyka i układ stawów](#kinematyka-i-układ-stawów)
4. [Konfiguracja dla G1](#konfiguracja-dla-g1)
5. [Pierwsze kroki z G1](#pierwsze-kroki-z-g1)
6. [Sterowanie ramionami](#sterowanie-ramionami)
7. [Sterowanie nogami](#sterowanie-nogami)
8. [Zaawansowane funkcje](#zaawansowane-funkcje)
9. [Bezpieczeństwo pracy](#bezpieczeństwo-pracy)

---

## Wprowadzenie do G1 EDU

### Czym jest Unitree G1 EDU?

**Unitree G1 EDU** to zaawansowany **robot humanoidalny** przeznaczony do badań i edukacji w zakresie robotyki humanoidalnej. Jest to pełnowymiarowy robot dwunożny z ramionami, zdolny do:

- 🚶 Chodzenia i utrzymywania równowagi
- 🤲 Manipulacji obiektów za pomocą ramion
- 🎯 Interakcji z otoczeniem
- 🔬 Realizacji zaawansowanych projektów badawczych

### Dlaczego roboty humanoidalne są wyjątkowe?

Roboty humanoidalne, w przeciwieństwie do robotów kołowych lub czworonożnych, mają:
- **Wyższy środek ciężkości** → trudniejsze balansowanie
- **Mniej punktów kontaktu** (2 stopy vs 4 łapy) → wymaga zaawansowanego sterowania
- **Manipulatory (ramiona)** → możliwość interakcji z obiektami zaprojektowanymi dla ludzi
- **Podobną do człowieka kinematykę** → może działać w środowisku ludzkim

### Różnice między G1 a Go2/B2

| Właściwość | Go2/B2 (Quadruped) | G1 (Humanoid) |
|------------|-------------------|---------------|
| **Liczba nóg** | 4 | 2 |
| **Ramiona** | Brak | 2 ramiona (7 DoF każde) |
| **Liczba silników** | 12 | 23/29 (zależnie od wersji) |
| **Stabilność** | Wysoka (4 punkty podparcia) | Wymaga aktywnego balansowania |
| **Typ wiadomości** | `unitree_go::msg` | `unitree_hg::msg` |
| **Zastosowania** | Transport, inspekcje | Manipulacja, interakcje, badania |

---

## Specyfikacja robota

### Unitree G1 EDU - Wersje

G1 występuje w różnych konfiguracjach:

1. **G1 23DoF** - Podstawowa wersja edukacyjna
   - 12 stawów nóg (po 6 na nogę)
   - 10 stawów ramion (po 5 na ramię)
   - 1 staw pasa (yaw)
   
2. **G1 29DoF** - Pełna wersja
   - 12 stawów nóg
   - 14 stawów ramion (po 7 na ramię, z nadgarstkami)
   - 3 stawy pasa (yaw, roll, pitch)

### Parametry techniczne

```
Wysokość: ~1.3 m
Waga: ~35 kg
Stopnie swobody (DoF): 23 lub 29
Czas pracy: ~2 godziny (zależnie od zastosowania)
Komunikacja: CycloneDDS (Ethernet)
Częstotliwość sterowania: 500 Hz
Czujniki:
  - IMU (Inertial Measurement Unit)
  - Enkodery w każdym stawie
  - Czujniki siły w stopach
  - (opcjonalnie) Kamery, LiDAR
```

---

## Kinematyka i układ stawów

### Mapa stawów G1

Robot G1 ma **29 potencjalnych stawów** (niektóre mogą być nieaktywne w wersji 23DoF):

```
                    [Głowa - opcjonalna]
                           |
            ╔══════════════╩══════════════╗
            ║         TUŁÓW (Waist)       ║
            ║   [12] Yaw  [13] Roll*      ║
            ║         [14] Pitch*         ║
            ╚═══════╦═══════╦═════════════╝
                    ║       ║
         ┌──────────┘       └──────────┐
         │                              │
    LEWE RAMIĘ                    PRAWE RAMIĘ
    [15] Shoulder Pitch           [22] Shoulder Pitch
    [16] Shoulder Roll            [23] Shoulder Roll
    [17] Shoulder Yaw             [24] Shoulder Yaw
    [18] Elbow                    [25] Elbow
    [19] Wrist Roll               [26] Wrist Roll
    [20] Wrist Pitch*             [27] Wrist Pitch*
    [21] Wrist Yaw*               [28] Wrist Yaw*

              ┌──────┴──────┐
              │             │
         LEWA NOGA      PRAWA NOGA
         [0] Hip Pitch  [6] Hip Pitch
         [1] Hip Roll   [7] Hip Roll
         [2] Hip Yaw    [8] Hip Yaw
         [3] Knee       [9] Knee
         [4] Ankle Pitch [10] Ankle Pitch
         [5] Ankle Roll  [11] Ankle Roll
         
* Nieaktywne w wersji 23DoF z zablokowanym pasem
```

### Indeksy stawów w kodzie

Dla łatwiejszego odwoływania się do stawów, używamy enuma:

```cpp
enum G1JointIndex {
  // Lewa noga (0-5)
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,  // lub LEFT_ANKLE_B w trybie PR
  LEFT_ANKLE_ROLL = 5,   // lub LEFT_ANKLE_A w trybie PR
  
  // Prawa noga (6-11)
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,  // lub RIGHT_ANKLE_B w trybie PR
  RIGHT_ANKLE_ROLL = 11,   // lub RIGHT_ANKLE_A w trybie PR
  
  // Pas (12-14) - UWAGA: Może być zablokowany!
  WAIST_YAW = 12,
  WAIST_ROLL = 13,    // NIEAKTYWNY w G1 23DoF/29DoF z zablokowanym pasem
  WAIST_PITCH = 14,   // NIEAKTYWNY w G1 23DoF/29DoF z zablokowanym pasem
  
  // Lewe ramię (15-21)
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_ROLL = 19,
  LEFT_WRIST_PITCH = 20,   // NIEAKTYWNY w G1 23DoF
  LEFT_WRIST_YAW = 21,     // NIEAKTYWNY w G1 23DoF
  
  // Prawe ramię (22-28)
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_ROLL = 26,
  RIGHT_WRIST_PITCH = 27,  // NIEAKTYWNY w G1 23DoF
  RIGHT_WRIST_YAW = 28     // NIEAKTYWNY w G1 23DoF
};
```

### Zakresy ruchów stawów

⚠️ **Ważne:** Przekroczenie zakresów może uszkodzić robota!

**Przykładowe bezpieczne zakresy (wartości orientacyjne):**

```cpp
// Biodra (Hip)
// Pitch: -1.0 do 1.5 rad (~-57° do 86°)
// Roll: -0.5 do 0.5 rad (~-29° do 29°)
// Yaw: -0.5 do 0.5 rad

// Kolana (Knee)
// 0.0 do 2.5 rad (0° do ~143°)

// Kostki (Ankle)
// Pitch: -0.5 do 0.5 rad
// Roll: -0.3 do 0.3 rad

// Ramiona - zostaną podane w sekcji o ramionach
```

💡 **Wskazówka:** Zawsze sprawdź dokumentację techniczną dla dokładnych zakresów!

---

## Konfiguracja dla G1

### 1. Typ wiadomości

G1 używa **innej przestrzeni nazw** niż Go2/B2:

```cpp
// Dla Go2/B2:
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"

// Dla G1/H1:
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
```

### 2. Liczba silników

```cpp
// Go2/B2 ma 12 silników
const int GO2_NUM_MOTORS = 12;

// G1 ma 29 miejsc na silniki (niektóre mogą być nieaktywne)
const int G1_NUM_MOTOR = 29;
```

### 3. Funkcje CRC

```cpp
// Dla Go2/B2:
#include "common/motor_crc.h"
get_crc(unitree_go::msg::LowCmd& cmd);

// Dla G1/H1:
#include "common/motor_crc_hg.h"
get_crc(unitree_hg::msg::LowCmd& cmd);
```

### 4. Tryby pracy kostki (Ankle)

G1 ma specjalną cechę - kostki mogą pracować w dwóch trybach:

**Tryb AB (Pitch/Roll - domyślny):**
```cpp
low_cmd.mode_pr = PRorAB::AB;  // Tryb Ankle_B / Ankle_A
// [4] i [5] = Ankle Pitch/Roll dla lewej nogi
// [10] i [11] = Ankle Pitch/Roll dla prawej nogi
```

**Tryb PR (alternatywny):**
```cpp
low_cmd.mode_pr = PRorAB::PR;  // Tryb specjalny
// Inna interpretacja stawów kostki
```

**Dla większości zastosowań używaj trybu AB.**

---

## Pierwsze kroki z G1

### Program 1: Odczyt stanu G1

**Cel:** Odczytać stan wszystkich 29 stawów G1.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_state.hpp"

class G1StateReader : public rclcpp::Node {
public:
    G1StateReader() : Node("g1_state_reader") {
        // G1 używa tych samych nazw topików co Go2
        subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            "lowstate",  // lub "lf/lowstate" dla niskiej częstotliwości
            10,
            [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                this->print_state(msg);
            }
        );
    }

private:
    void print_state(const unitree_hg::msg::LowState::SharedPtr msg) {
        // Wyświetl informacje o mode_machine
        RCLCPP_INFO(this->get_logger(), "=== Stan G1 ===");
        RCLCPP_INFO(this->get_logger(), "Mode machine: %d", msg->mode_machine);
        
        // IMU
        RCLCPP_INFO(this->get_logger(), "IMU Roll: %.2f° Pitch: %.2f° Yaw: %.2f°",
                   msg->imu_state.rpy[0] * 180.0 / M_PI,
                   msg->imu_state.rpy[1] * 180.0 / M_PI,
                   msg->imu_state.rpy[2] * 180.0 / M_PI);
        
        // Przykładowe stawy
        RCLCPP_INFO(this->get_logger(), 
                   "Lewe biodro pitch: %.3f rad (%.1f°)",
                   msg->motor_state[0].q,
                   msg->motor_state[0].q * 180.0 / M_PI);
        
        RCLCPP_INFO(this->get_logger(),
                   "Lewy łokieć: %.3f rad (%.1f°)",
                   msg->motor_state[18].q,
                   msg->motor_state[18].q * 180.0 / M_PI);
        
        // Bateria
        RCLCPP_INFO(this->get_logger(),
                   "Bateria: %.1fV, %.2fA",
                   msg->power_v, msg->power_a);
    }
    
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<G1StateReader>());
    rclcpp::shutdown();
    return 0;
}
```

### Program 2: Zero Posture - Przejście do pozycji zerowej

**Cel:** Bezpiecznie przesunąć wszystkie stawy do pozycji neutralnej (0°).

**Dlaczego to ważne?** 
- Znana pozycja startowa dla dalszych eksperymentów
- Test czy wszystkie silniki odpowiadają
- Trening bezpiecznego sterowania

```cpp
#include "common/motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

class G1ZeroPosture : public rclcpp::Node {
public:
    G1ZeroPosture() : Node("g1_zero_posture") {
        // Subskrybent stanu
        state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
            "lowstate", 10,
            [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                this->state_callback(msg);
            }
        );
        
        // Wydawca komend
        cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
        
        // Timer do wysyłania komend z częstotliwością 500Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),  // 2ms = 500Hz
            [this]() { this->control_loop(); }
        );
        
        RCLCPP_INFO(this->get_logger(), "G1 Zero Posture - Start");
        RCLCPP_INFO(this->get_logger(), "Robot będzie powoli przechodzić do pozycji zerowej");
        
        start_time_ = this->now();
        duration_ = 3.0;  // 3 sekundy na przejście
    }

private:
    void state_callback(const unitree_hg::msg::LowState::SharedPtr msg) {
        // Zapisz aktualny stan
        current_state_ = *msg;
        state_received_ = true;
    }
    
    void control_loop() {
        if (!state_received_) {
            // Czekaj na pierwszy stan
            return;
        }
        
        // Oblicz czas od startu
        double elapsed = (this->now() - start_time_).seconds();
        
        // Oblicz współczynnik interpolacji (0.0 → 1.0)
        double ratio = std::min(elapsed / duration_, 1.0);
        
        // Przygotuj komendę
        unitree_hg::msg::LowCmd cmd;
        cmd.mode_pr = 1;  // Tryb AB
        cmd.mode_machine = 0;
        
        // Dla każdego silnika
        for (int i = 0; i < 29; i++) {
            // Pobierz aktualną pozycję z pierwszego odczytu
            if (elapsed < 0.01) {  // Pierwszy raz
                initial_positions_[i] = current_state_.motor_state[i].q;
            }
            
            // INTERPOLACJA: stopniowo przechodź z initial_position do 0.0
            double target_q = (1.0 - ratio) * initial_positions_[i] + ratio * 0.0;
            
            cmd.motor_cmd[i].mode = 1;      // Włącz silnik
            cmd.motor_cmd[i].q = target_q;  // Docelowa pozycja
            cmd.motor_cmd[i].dq = 0.0;      // Bez ruchu
            cmd.motor_cmd[i].tau = 0.0;     // Bez dodatkowego momentu
            
            // Różne wzmocnienia dla różnych części ciała
            if (i < 12) {
                // Nogi - mocniejsze wzmocnienie (utrzymują ciężar)
                cmd.motor_cmd[i].kp = 100.0;
                cmd.motor_cmd[i].kd = 2.0;
            } else if (i >= 15 && i <= 28) {
                // Ramiona - słabsze wzmocnienie
                cmd.motor_cmd[i].kp = 50.0;
                cmd.motor_cmd[i].kd = 1.0;
            } else {
                // Pas i inne
                cmd.motor_cmd[i].kp = 50.0;
                cmd.motor_cmd[i].kd = 1.0;
            }
        }
        
        // Oblicz CRC i wyślij
        get_crc(cmd);
        cmd_pub_->publish(cmd);
        
        // Informuj użytkownika
        if (static_cast<int>(elapsed * 10) % 10 == 0) {  // Co 1 sekundę
            RCLCPP_INFO(this->get_logger(), 
                       "Progress: %.0f%% (%.1fs / %.1fs)",
                       ratio * 100.0, elapsed, duration_);
        }
        
        if (elapsed >= duration_ + 1.0) {
            RCLCPP_INFO(this->get_logger(), "✓ Pozycja zerowa osiągnięta!");
            // Możesz zatrzymać timer lub kontynuować utrzymywanie pozycji
        }
    }
    
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    unitree_hg::msg::LowState current_state_;
    bool state_received_ = false;
    
    rclcpp::Time start_time_;
    double duration_;
    std::array<double, 29> initial_positions_ = {0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<G1ZeroPosture>());
    rclcpp::shutdown();
    return 0;
}
```

**Kluczowe koncepcje w tym przykładzie:**

1. **Interpolacja:** Stopniowe przejście od pozycji aktualnej do docelowej
   ```
   target = (1 - ratio) * start + ratio * end
   ratio: 0.0 → 1.0 w czasie duration
   ```

2. **Częstotliwość 500Hz:** Timer co 2ms dla płynnego sterowania

3. **Różne wzmocnienia:** Nogi vs ramiona - różne obciążenia

4. **CRC:** Zawsze przed wysłaniem!

---

## Sterowanie ramionami

### Anatomia ramienia G1

Każde ramię ma **5 lub 7 stawów** (zależnie od wersji):

```
         Bark (Shoulder)
              │
    ┌─────────┼─────────┐
    │ [15/22] Pitch     │  ← Podnoszenie ramienia
    │ [16/23] Roll      │  ← Oddalanie od ciała
    │ [17/24] Yaw       │  ← Obrót ramienia
    │                   │
    │     Łokieć (Elbow)│
    │    [18/25]        │  ← Zgięcie łokcia
    │                   │
    │   Nadgarstek      │
    │  [19/26] Roll     │  ← Obrót nadgarstka
    │  [20/27] Pitch*   │  ← Zgięcie w górę/dół
    │  [21/28] Yaw*     │  ← Obrót dłoni
    └───────────────────┘
    
* Tylko w wersji 29DoF
```

### Przykład: Podnoszenie ramion

**Zadanie:** Podnieś oba ramiona do pozycji horyzontalnej.

```cpp
class G1ArmControl : public rclcpp::Node {
    // ... inicjalizacja jak w poprzednich przykładach ...
    
    void control_loop() {
        if (!state_received_) return;
        
        double elapsed = (this->now() - start_time_).seconds();
        double ratio = std::min(elapsed / 3.0, 1.0);  // 3 sekundy
        
        unitree_hg::msg::LowCmd cmd;
        cmd.mode_pr = 1;
        cmd.mode_machine = 0;
        
        // Inicjalizuj wszystkie silniki
        for (int i = 0; i < 29; i++) {
            cmd.motor_cmd[i].mode = 1;
            cmd.motor_cmd[i].q = 0.0;
            cmd.motor_cmd[i].dq = 0.0;
            cmd.motor_cmd[i].tau = 0.0;
            cmd.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
            cmd.motor_cmd[i].kd = 1.0;
        }
        
        // === STEROWANIE RAMIONAMI ===
        
        // Docelowe kąty (w radianach)
        double target_shoulder_pitch = M_PI / 2.0;  // 90° = ramiona w poziomie
        double target_elbow = 0.0;                   // Proste łokcie
        
        // LEWE RAMIĘ
        cmd.motor_cmd[LEFT_SHOULDER_PITCH].q = ratio * target_shoulder_pitch;
        cmd.motor_cmd[LEFT_SHOULDER_ROLL].q = 0.3 * ratio;  // Lekko na boki
        cmd.motor_cmd[LEFT_ELBOW].q = ratio * target_elbow;
        
        // PRAWE RAMIĘ (symetrycznie)
        cmd.motor_cmd[RIGHT_SHOULDER_PITCH].q = ratio * target_shoulder_pitch;
        cmd.motor_cmd[RIGHT_SHOULDER_ROLL].q = -0.3 * ratio;  // Lustrzane odbicie
        cmd.motor_cmd[RIGHT_ELBOW].q = ratio * target_elbow;
        
        get_crc(cmd);
        cmd_pub_->publish(cmd);
        
        if (elapsed >= 3.0 && elapsed < 3.1) {
            RCLCPP_INFO(this->get_logger(), "✓ Ramiona podniesione!");
        }
    }
};
```

### Przykład: Machanie ręką

**Zadanie:** Cykliczny ruch ramienia (machanie).

```cpp
void wave_hand() {
    // Czas od startu
    double t = (this->now() - start_time_).seconds();
    
    // Generuj sinusoidalną trajektorię
    double shoulder_pitch = M_PI / 3.0;  // Stała wysokość ramienia (~60°)
    double elbow_angle = M_PI / 4.0 * (1.0 + std::sin(2.0 * M_PI * 0.5 * t));
    //                   └─ amplituda ─┘  └─ sinus ─┘ └─ częstotliwość 0.5Hz ─┘
    
    // Łokieć będzie się zginał i prostował: 0° → 90° → 0° → ...
    
    unitree_hg::msg::LowCmd cmd;
    // ... inicjalizacja ...
    
    // Prawe ramię
    cmd.motor_cmd[RIGHT_SHOULDER_PITCH].q = shoulder_pitch;
    cmd.motor_cmd[RIGHT_ELBOW].q = elbow_angle;
    
    get_crc(cmd);
    cmd_pub_->publish(cmd);
}
```

**Koncepcja:**
- Używamy funkcji `sin()` do stworzenia płynnego, cyklicznego ruchu
- Częstotliwość 0.5Hz = jedno machnięcie co 2 sekundy
- Można zmienić częstotliwość, amplitudę, fazę dla różnych efektów

---

## Sterowanie nogami

### Uwaga o bezpieczeństwie! ⚠️

**Sterowanie nogami robota humanoidalnego jest BARDZO delikatne!**

- Robot może stracić równowagę i upaść
- Upadek może uszkodzić sprzęt
- NIE eksperymentuj bez nadzoru lub zabezpieczeń

**Zalecenia:**
1. Zawieś robota na lince/stojaku podczas testów
2. Zacznij od bardzo małych ruchów
3. Zawsze miej kontroler awaryjny w ręku

### Anatomia nogi G1

```
         Biodro (Hip)
             │
    ┌────────┼────────┐
    │ [0/6]  Pitch    │  ← Ruch do przodu/tyłu
    │ [1/7]  Roll     │  ← Ruch na boki
    │ [2/8]  Yaw      │  ← Obrót nogi
    │                 │
    │    Kolano       │
    │    [3/9]        │  ← Zgięcie kolana
    │                 │
    │    Kostka       │
    │ [4/10] Pitch    │  ← Zgięcie stopy
    │ [5/11] Roll     │  ← Przechylenie stopy
    └─────────────────┘
```

### Przykład: Podnoszenie nogi (z zawieszeniem!)

**TYLKO gdy robot jest bezpiecznie zawieszony!**

```cpp
void lift_leg_safely() {
    double t = (this->now() - start_time_).seconds();
    double ratio = std::min(t / 2.0, 1.0);  // 2 sekundy
    
    unitree_hg::msg::LowCmd cmd;
    // ... inicjalizacja ...
    
    // Stawy nóg wymagają WYSOKICH wzmocnień (niosą ciężar!)
    for (int i = 0; i < 12; i++) {
        cmd.motor_cmd[i].kp = 120.0;
        cmd.motor_cmd[i].kd = 2.5;
    }
    
    // Podnoszenie lewej nogi (tylko gdy robot jest zawieszony!)
    double target_hip_pitch = 0.5;   // ~30° - podnieś nogę do przodu
    double target_knee = 1.0;        // ~60° - zegnij kolano
    
    cmd.motor_cmd[LEFT_HIP_PITCH].q = ratio * target_hip_pitch;
    cmd.motor_cmd[LEFT_KNEE].q = ratio * target_knee;
    
    // Kostka kompensuje, aby stopa była poziomo
    cmd.motor_cmd[LEFT_ANKLE_PITCH].q = -ratio * (target_hip_pitch + target_knee) / 2.0;
    
    get_crc(cmd);
    cmd_pub_->publish(cmd);
}
```

**Dlaczego kostka kompensuje?**
- Gdy zginamy biodro i kolano, stopa by opadła
- Chcemy, aby stopa pozostała poziomo
- Kostka musi "odwrócić" sumę kątów biodra i kolana

---

## Zaawansowane funkcje

### 1. High-Level API dla G1

G1 ma wysokopoziomowe API do sterowania ramionami i ruchem:

```cpp
#include "unitree_api/msg/request.hpp"

// Przykład: Sterowanie ramieniem przez API
class G1ArmAPI : public rclcpp::Node {
    // Publikuj do /api/arm/request (upewnij się, jaki topik używa G1)
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr arm_req_pub_;
    
    void send_arm_command() {
        unitree_api::msg::Request req;
        // ... Stwórz odpowiedni Request dla API ramienia ...
        arm_req_pub_->publish(req);
    }
};
```

**Sprawdź dostępne topiki API:**
```bash
ros2 topic list | grep api
```

### 2. Dex3 - Dexterous Hand (opcjonalnie)

Niektóre wersje G1 mogą mieć zaawansowane dłonie:

```bash
# Przykład dla Dex3
./install/unitree_ros2_example/bin/g1_dex3_example
```

### 3. Audio Control

G1 może mieć interfejs audio:

```bash
# Przykład kontroli audio
./install/unitree_ros2_example/bin/g1_audio_client_example
```

### 4. Locomotion Client - Chodzenie

```bash
# Wysokopoziomowe sterowanie chodzeniem
./install/unitree_ros2_example/bin/loco_client_example
```

---

## Bezpieczeństwo pracy

### Procedura bezpiecznego testowania

**Przed każdym testem:**

1. ✅ Sprawdź baterię (minimum 30%)
2. ✅ Upewnij się, że wokół jest dużo wolnej przestrzeni
3. ✅ Miej kontroler awaryjny w zasięgu ręki
4. ✅ Dla testów nóg - zawieś robota
5. ✅ Przeczytaj kod - zrozum co robi
6. ✅ Zacznij od małych wartości kp/kd

**Podczas testu:**

- 👀 Obserwuj robota cały czas
- 🛑 Trzymaj palec na przycisku awaryjnym
- 📊 Monitoruj temperatury silników
- 🔋 Sprawdzaj poziom baterii

**Po teście:**

- 📝 Zapisz wyniki i obserwacje
- 💾 Nagraj dane (ros2 bag)
- 🔧 Sprawdź czy wszystko działa poprawnie

### Sygnały ostrzegawcze

**NATYCHMIAST ZATRZYMAJ gdy:**

- 🔴 Robot zachowuje się niestabilnie
- 🔴 Silniki są gorące (>70°C)
- 🔴 Dziwne dźwięki z silników
- 🔴 Bateria poniżej 20%
- 🔴 Komunikaty błędów w logach

### Bezpieczne wartości startowe

```cpp
// Dla eksperymentów rozpocznij od:
const double SAFE_KP_LEGS = 80.0;   // Nogi
const double SAFE_KD_LEGS = 1.5;

const double SAFE_KP_ARMS = 40.0;   // Ramiona
const double SAFE_KD_ARMS = 0.8;

const double MAX_JOINT_SPEED = 2.0; // rad/s - bezpieczna prędkość
```

---

## Podsumowanie i następne kroki

**Czego się nauczyłeś:**
- ✅ Specyfika robota humanoidalnego G1 EDU
- ✅ Układ stawów i kinematyka
- ✅ Różnice między G1 a Go2/B2
- ✅ Sterowanie ramionami
- ✅ Podstawy sterowania nogami
- ✅ Zasady bezpieczeństwa

**Zalecane projekty dla G1:**
- 🤖 Gestykulacja - zaprogramuj naturalne gesty
- 🎯 Wskazywanie - robot pokazuje obiekty
- 📦 Manipulacja - podnoszenie i przenoszenie obiektów
- 🚶 Koordynacja - równoczesne ruchy ramion i nóg
- 👋 Interakcje społeczne - machanie, uścisk dłoni

**Dalsze materiały:**
- [Przykłady projektów G1](PRZYKLADY_PROJEKTOW_G1.md)
- Oficjalna dokumentacja: https://support.unitree.com/
- Przykłady w `example/src/src/g1/`

---

**Bezpiecznej pracy z robotem G1! 🤖🎓**
