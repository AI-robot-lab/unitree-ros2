# Przykładowe Projekty dla Robota Unitree G1 EDU

## 📖 Spis treści
1. [Projekty podstawowe](#projekty-podstawowe)
2. [Projekty średnio-zaawansowane](#projekty-średnio-zaawansowane)
3. [Projekty zaawansowane](#projekty-zaawansowane)
4. [Projekty badawcze](#projekty-badawcze)
5. [Wskazówki realizacji](#wskazówki-realizacji)

---

## Projekty podstawowe

### Projekt 1: Dashboard Monitorujący 📊

**Poziom:** 🟢 Podstawowy  
**Czas realizacji:** 1-2 tygodnie  
**Umiejętności:** Odczyt danych, wizualizacja, interfejsy użytkownika

#### Opis
Stwórz aplikację z interfejsem graficznym, która w czasie rzeczywistym wyświetla:
- Stan wszystkich 29 stawów (pozycje, prędkości, temperatury)
- Dane z IMU (orientacja, przyspieszenia)
- Stan baterii
- Ostrzeżenia o przekroczeniu limitów

#### Cele edukacyjne
- ✅ Nauka subskrypcji topików ROS2
- ✅ Przetwarzanie strumienia danych
- ✅ Tworzenie GUI (np. Qt, web interface)
- ✅ Wizualizacja danych w czasie rzeczywistym

#### Sugerowane technologie
- **Backend:** ROS2 + Python lub C++
- **Frontend:** 
  - PyQt5/PySide6 (Python GUI)
  - RViz2 (wbudowane narzędzie ROS2)
  - Web: React + rosbridge
- **Wykresy:** Matplotlib, Plotly

#### Kamienie milowe
1. **Tydzień 1:**
   - Odczyt danych z topiku `/lowstate`
   - Wyświetlenie danych w konsoli
   - Prosty GUI z podstawowymi informacjami

2. **Tydzień 2:**
   - Wykresy w czasie rzeczywistym
   - System ostrzeżeń
   - Zapis danych do pliku

#### Rozszerzenia
- 📈 Historia danych (wykresy 3D pozycji stawów)
- 🎨 Wizualizacja 3D robota (URDF model)
- 🔔 Powiadomienia push o krytycznych stanach
- 💾 Eksport danych do CSV/JSON

---

### Projekt 2: Kontroler Gestów 🤲

**Poziom:** 🟢 Podstawowy  
**Czas realizacji:** 2-3 tygodnie  
**Umiejętności:** Sterowanie, interpolacja trajektorii

#### Opis
Zaprogramuj robota do wykonywania zestawu prostych gestów:
- 👋 Machanie ręką na powitanie
- 👍 Kciuk w górę (aprobuję)
- 👎 Kciuk w dół (dezaprobata)
- 🤝 Gest podania ręki
- 🙋 Podniesienie ręki

#### Cele edukacyjne
- ✅ Sterowanie ramionami robota
- ✅ Planowanie trajektorii
- ✅ Interpolacja między pozycjami
- ✅ Tworzenie biblioteki ruchów

#### Architektura rozwiązania
```
┌──────────────────┐
│  Interfejs       │ ← Wybór gestu przez użytkownika
│  (CLI lub GUI)   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  Gesture Library │ ← Definicje gestów (JSON/YAML)
│  (baza gestów)   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Trajectory       │ ← Generowanie płynnej trajektorii
│ Planner          │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Low-Level        │ ← Wysyłanie komend do robota
│ Controller       │
└──────────────────┘
```

#### Przykładowa definicja gestu (YAML)
```yaml
wave_hand:
  name: "Machanie ręką"
  duration: 3.0  # sekundy
  arm: "right"   # lewe/prawe ramię
  keyframes:
    - time: 0.0
      joints:
        shoulder_pitch: 1.57  # 90° (ramię w górę)
        shoulder_roll: 0.0
        elbow: 0.0
    - time: 1.0
      joints:
        shoulder_pitch: 1.57
        elbow: 1.0  # Zegnij łokieć
    - time: 2.0
      joints:
        shoulder_pitch: 1.57
        elbow: 0.0  # Wyprostuj
    - time: 3.0
      joints:
        shoulder_pitch: 0.0  # Opuść ramię
        elbow: 0.0
```

#### Kamienie milowe
1. **Tydzień 1:**
   - Podstawowe sterowanie ramionami
   - Jeden prosty gest (machanie)
   - Interpolacja liniowa między punktami

2. **Tydzień 2:**
   - Biblioteka 5 gestów
   - Ładowanie gestów z pliku YAML
   - Interfejs wyboru gestu

3. **Tydzień 3:**
   - Płynniejsza interpolacja (kubiczna)
   - Synchronizacja obu ramion
   - Możliwość nagrywania własnych gestów

#### Rozszerzenia
- 🎮 Sterowanie gestami przez kontroler
- 🗣️ Aktywacja głosowa (integracja z ROS audio)
- 📹 Rozpoznawanie gestów ludzkich (kamera + CV)
- 🎭 Ekspresja całego ciała (+ nogi)

---

### Projekt 3: Teleoperation z Kontrolera 🎮

**Poziom:** 🟢 Podstawowy  
**Czas realizacji:** 2 tygodnie  
**Umiejętności:** Odczyt kontrolera, mapowanie wartości

#### Opis
Zaimplementuj intuicyjne sterowanie robotem używając bezprzewodowego kontrolera Unitree:
- **Lewy joystick:** Kontrola pozycji korpusu (pochylenie)
- **Prawy joystick:** Sterowanie ramieniem
- **Przyciski:** Przełączanie trybów, wykonanie gestów

#### Cele edukacyjne
- ✅ Odczyt danych z kontrolera
- ✅ Mapowanie analogowych sygnałów
- ✅ Implementacja stref martwych
- ✅ Zmiana trybów sterowania

#### Mapa przypisań kontrolera
```
Kontroler Unitree:

┌─────────────────────────────────────────────────┐
│  [L2]                             [R2]           │
│  [L1]                             [R1]           │
│                                                  │
│    ╔═══╗                           (Y)           │
│    ║ L ║                      (X)  [A]  (B)      │
│    ╚═══╝                                         │
│                                   ╔═══╗          │
│                                   ║ R ║          │
│                                   ╚═══╝          │
│                                                  │
│  [SELECT]                     [START]            │
└─────────────────────────────────────────────────┘

Proponowane mapowanie:
- L joystick: Pochylenie korpusu (pitch/roll)
- R joystick: Sterowanie prawym ramieniem
- L1: Przełącz na lewe ramię
- R1: Przełącz na prawe ramię
- A: Pozycja zero
- B: Zapisz pozycję
- X: Odtwórz pozycję
- Y: Tryb awaryjny (disable)
- Start: Włącz/wyłącz sterowanie
```

#### Kamienie milowe
1. **Tydzień 1:**
   - Odczyt wszystkich przycisków i joysticków
   - Sterowanie jednym ramieniem
   - Strefa martwa i skalowanie

2. **Tydzień 2:**
   - Przełączanie trybów
   - Sterowanie pozycją korpusu
   - Zabezpieczenia i limity

#### Rozszerzenia
- 🎮 Wsparcie dla innych kontrolerów (Xbox, PS4)
- 📝 Konfigurowalny mapping (plik konfiguracyjny)
- 🎚️ Regulowana czułość joysticków
- 🔊 Feedback dźwiękowy/haptyczny

---

## Projekty średnio-zaawansowane

### Projekt 4: Object Pointing System 🎯

**Poziom:** 🟡 Średnio-zaawansowany  
**Czas realizacji:** 3-4 tygodnie  
**Umiejętności:** Kinematyka, transformacje, percepcja

#### Opis
Robot wykrywa obiekty w przestrzeni 3D i wskazuje je palcem/ramieniem. System potrafi:
- Wykryć pozycję obiektu (kamera RGB-D lub LiDAR)
- Obliczyć kinematykę odwrotną ramienia
- Wskazać obiekt palcem z precyzją

#### Cele edukacyjne
- ✅ Transformacje przestrzenne (TF2)
- ✅ Kinematyka odwrotna (Inverse Kinematics)
- ✅ Integracja z sensorami
- ✅ Koordynacja oko-ręka

#### Architektura systemu
```
┌─────────────┐
│   Kamera    │ ← Wykrywa obiekt (3D point cloud)
│   lub LiDAR │
└──────┬──────┘
       │ Pozycja obiektu w układzie kamery
       ▼
┌─────────────┐
│ TF2 Trans-  │ ← Transformacja do układu robota
│ formation   │
└──────┬──────┘
       │ Pozycja obiektu w układzie bazowym
       ▼
┌─────────────┐
│  Inverse    │ ← Obliczenie kątów stawów
│ Kinematics  │
└──────┬──────┘
       │ Kąty stawów [θ1, θ2, ...]
       ▼
┌─────────────┐
│   Motion    │ ← Płynna trajektoria
│  Planning   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    G1       │ ← Wykonanie ruchu
│   Robot     │
└─────────────┘
```

#### Etapy realizacji

**Faza 1: Symulacja (tydzień 1-2)**
- Wirtualne obiekty w RViz
- Kinematyka odwrotna dla ramienia G1
- Testy z różnymi pozycjami

**Faza 2: Percepcja (tydzień 2-3)**
- Integracja z kamerą/LiDARem
- Detekcja obiektów (np. ArUco markers)
- Transformacje TF2

**Faza 3: Integracja (tydzień 3-4)**
- Połączenie percepcji z IK
- Testy na prawdziwym robocie
- Optymalizacja precyzji

#### Narzędzia matematyczne

**Kinematyka prosta (Forward Kinematics):**
```
Dany: kąty stawów [θ1, θ2, θ3, θ4, θ5]
Znajdź: pozycja końcówki ramienia [x, y, z]
```

**Kinematyka odwrotna (Inverse Kinematics):**
```
Dany: docelowa pozycja [x, y, z]
Znajdź: kąty stawów [θ1, θ2, θ3, θ4, θ5]
```

**Biblioteki pomocnicze:**
- **KDL (Kinematics and Dynamics Library)** - solwer IK
- **MoveIt2** - planowanie ruchu
- **tf2** - transformacje przestrzenne

#### Przykład użycia tf2
```cpp
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Transformacja punktu z układu kamery do układu bazowego robota
geometry_msgs::msg::PointStamped point_camera;
point_camera.header.frame_id = "camera_link";
point_camera.point.x = 0.5;  // Punkt wykryty przez kamerę

geometry_msgs::msg::PointStamped point_base;
tf_buffer->transform(point_camera, point_base, "base_link");
// Teraz point_base zawiera współrzędne w układzie robota
```

#### Rozszerzenia
- 🤖 Śledzenie ruchomych obiektów
- 👀 Koordynacja głowy (jeśli G1 ma ruchomą głowę)
- 📍 Dotknięcie obiektu (reach and touch)
- 🗣️ Interakcja głosowa "wskaż stół", "wskaż drzwi"

---

### Projekt 5: Pick and Place 📦

**Poziom:** 🟡 Średnio-zaawansowany  
**Czas realizacji:** 4-5 tygodni  
**Umiejętności:** Manipulacja, planowanie trajektorii, chwytak

#### Opis
Pełny pipeline do podnoszenia i przenoszenia obiektów:
1. Wykryj obiekt
2. Zaplanuj trajektorię do obiektu
3. Chwycenie (jeśli G1 ma dłoń/chwytak)
4. Przeniesienie do celu
5. Uwolnienie

#### Cele edukacyjne
- ✅ Planowanie trajektorii (path planning)
- ✅ Unikanie kolizji
- ✅ Sterowanie chwytakiem
- ✅ Koordynacja wielu systemów

#### State Machine
```
┌─────────┐
│  IDLE   │ ← Stan startowy
└────┬────┘
     │ Wykryto obiekt
     ▼
┌─────────┐
│  DETECT │ ← Identyfikacja i lokalizacja obiektu
└────┬────┘
     │ Pozycja znana
     ▼
┌─────────┐
│  PLAN   │ ← Planowanie trajektorii
└────┬────┘
     │ Trajektoria gotowa
     ▼
┌─────────┐
│  REACH  │ ← Ruch do obiektu
└────┬────┘
     │ Osiągnięto pozycję
     ▼
┌─────────┐
│  GRASP  │ ← Chwycenie
└────┬────┘
     │ Obiekt chwycony
     ▼
┌─────────┐
│  LIFT   │ ← Podniesienie
└────┬────┘
     │
     ▼
┌─────────┐
│  MOVE   │ ← Przeniesienie do celu
└────┬────┘
     │ Osiągnięto cel
     ▼
┌─────────┐
│ RELEASE │ ← Uwolnienie obiektu
└────┬────┘
     │
     ▼
┌─────────┐
│ RETURN  │ ← Powrót do pozycji startowej
└────┬────┘
     │
     └──→ IDLE
```

#### Kamienie milowe
1. **Tydzień 1-2:** Detekcja obiektów i IK
2. **Tydzień 3:** Planowanie trajektorii i unikanie kolizji
3. **Tydzień 4:** Integracja z chwytakiem (jeśli dostępny)
4. **Tydzień 5:** Testy end-to-end, optymalizacja

#### Rozszerzenia
- 🎯 Sortowanie obiektów po kolorze/kształcie
- 📐 Precyzyjne układanie (stacking)
- 🔄 Wielokrotne podnoszenie (assembly)
- 🧩 Rozwiązywanie prostych puzzli

---

### Projekt 6: Human Following 👤

**Poziom:** 🟡 Średnio-zaawansowany  
**Czas realizacji:** 3-4 tygodnie  
**Umiejętności:** Computer vision, nawigacja, tracking

#### Opis
Robot śledzi osobę w przestrzeni:
- Wykrywa osobę (kamera + detekcja człowieka)
- Orientuje się w jej kierunku
- Podąża za nią w bezpiecznej odległości
- Zatrzymuje się gdy osoba stoi

#### Cele edukacyjne
- ✅ Detekcja człowieka (YOLO, HOG)
- ✅ Tracking (filtr Kalmana)
- ✅ Sterowanie lokomocją
- ✅ Regulacja prędkości

#### Architektura
```
┌──────────┐
│  Camera  │ ← RGB lub RGB-D
└─────┬────┘
      │ Obraz
      ▼
┌──────────┐
│  Person  │ ← CNN detector (YOLO, MobileNet-SSD)
│ Detection│
└─────┬────┘
      │ Bounding box
      ▼
┌──────────┐
│ Distance │ ← Z depth camera lub lidar
│  Est.    │
└─────┬────┘
      │ Odległość i kąt
      ▼
┌──────────┐
│  PID     │ ← Regulacja: vx, vy, vyaw
│Controller│
└─────┬────┘
      │ Prędkości
      ▼
┌──────────┐
│  Sport   │ ← Sterowanie ruchem (jeśli G1 może chodzić)
│  Mode    │   lub wizualne śledzenie (obrót korpusu)
└──────────┘
```

#### Strategia sterowania

**Regulator PID dla orientacji:**
```cpp
// Błąd kątowy (osoba w lewo/prawo od centrum)
double error_angle = person_x - image_center_x;

// Regulator proporcjonalny
double vyaw = Kp * error_angle;

// Wyślij komendę ruchu
sport_client.Move(request, 0.0, 0.0, vyaw);
```

**Utrzymywanie odległości:**
```cpp
double desired_distance = 1.5;  // metry
double current_distance = depth_at_person;

double error_distance = current_distance - desired_distance;

// Jeśli za daleko - idź do przodu
// Jeśli za blisko - cofnij się
double vx = Kp_dist * error_distance;
```

#### Kamienie milowe
1. **Tydzień 1:** Detekcja osoby w obrazie
2. **Tydzień 2:** Estymacja odległości
3. **Tydzień 3:** Sterowanie orientacją (śledzenie wzrokiem)
4. **Tydzień 4:** Lokomocja (jeśli możliwa) lub zaawansowane śledzenie

#### Rozszerzenia
- 👥 Śledzenie konkretnej osoby (re-identification)
- 🚧 Unikanie przeszkód podczas podążania
- 🗣️ Interakcje głosowe podczas podążania
- 📸 Rozpoznawanie gestów osoby

---

## Projekty zaawansowane

### Projekt 7: Autonomous Navigation 🗺️

**Poziom:** 🔴 Zaawansowany  
**Czas realizacji:** 6-8 tygodni  
**Umiejętności:** SLAM, path planning, lokalizacja

#### Opis
Pełna autonomiczna nawigacja w środowisku:
- SLAM (budowanie mapy otoczenia)
- Lokalizacja na mapie
- Planowanie ścieżki
- Unikanie dynamicznych przeszkód

#### Technologie
- **Nav2** - ROS2 navigation stack
- **SLAM Toolbox** - budowanie map
- **AMCL** - lokalizacja
- **Costmap** - reprezentacja przeszkód

#### Rozszerzenia
- 🎯 Wykrywanie i omijanie osób
- 🚪 Otwieranie drzwi
- 🛗 Jazda windą
- 📡 Multi-robot coordination

---

### Projekt 8: Imitation Learning 🎓

**Poziom:** 🔴 Zaawansowany  
**Czas realizacji:** 8-10 tygodni  
**Umiejętności:** Machine learning, demonstracje, reinforcement learning

#### Opis
Robot uczy się zachowań poprzez obserwację demonstracji:
1. Nagrywanie trajektorii (kinesthetic teaching)
2. Uczenie modelu (behavioral cloning)
3. Odtwarzanie i generalizacja

#### Technologie
- **PyTorch/TensorFlow** - sieci neuronowe
- **ROS2 bag** - nagrywanie danych
- **DMPs (Dynamic Movement Primitives)** - reprezentacja trajektorii

#### Przykładowe zadania
- 🥤 Nalewanie wody do szklanki
- 🧹 Zamiatanie
- 📝 Pisanie

---

### Projekt 9: Dual-Arm Coordination 🤝

**Poziom:** 🔴 Zaawansowany  
**Czas realizacji:** 5-6 tygodni  
**Umiejętności:** Koordynacja, constraints, planowanie

#### Opis
Zadania wymagające współpracy obu ramion:
- Trzymanie długich obiektów dwoma rękami
- Otwieranie wieczka (jedna ręka trzyma, druga otwiera)
- Składanie pudełka
- Wiązanie supełka

#### Wyzwania
- Synchronizacja ruchów
- Ograniczenia (constraints) między ramionami
- Planowanie w przestrzeni konfiguracyjnej 10-14D

---

## Projekty badawcze

### Projekt 10: Balance Control 🎯

**Poziom:** 🔴 Zaawansowany/Badawczy  
**Czas realizacji:** 10+ tygodni  
**Tematyka:** Sterowanie, teoria systemów, optymalizacja

#### Opis
Implementacja zaawansowanego algorytmu balansowania dla robota humanoidalnego:
- Model Predictive Control (MPC)
- Whole-body control
- Zero Moment Point (ZMP) planning
- Compensation for external disturbances

#### Tematy badawcze
- 📊 Porównanie różnych algorytmów sterowania
- 🏃 Chodzenie po nierównym terenie
- 🤸 Reakcja na pchnięcia
- ⚖️ Optymalizacja zużycia energii

---

## Wskazówki realizacji

### Metodologia projektu

**1. Definicja wymagań**
- Co dokładnie ma robić system?
- Jakie są kryteria sukcesu?
- Jakie są ograniczenia?

**2. Dekompozycja**
- Podziel projekt na moduły
- Zdefiniuj interfejsy między modułami
- Zaplanuj zależności

**3. Iteracyjny rozwój**
- Zacznij od najprostszej wersji (MVP)
- Testuj często
- Stopniowo dodawaj funkcjonalności

**4. Testowanie**
- Testy jednostkowe (unit tests)
- Testy integracyjne
- Testy na prawdziwym sprzęcie

**5. Dokumentacja**
- README z instrukcjami
- Komentarze w kodzie
- Diagramy architektury
- Filmy demonstracyjne

### Struktura repozytorium projektu

```
my_g1_project/
├── README.md               ← Opis projektu
├── docs/
│   ├── architecture.md     ← Architektura systemu
│   ├── setup.md           ← Instrukcje instalacji
│   └── results.md         ← Wyniki eksperymentów
├── src/
│   ├── perception/        ← Moduł percepcji
│   ├── planning/          ← Moduł planowania
│   ├── control/           ← Moduł sterowania
│   └── utils/             ← Narzędzia pomocnicze
├── config/
│   ├── robot.yaml         ← Konfiguracja robota
│   └── parameters.yaml    ← Parametry algorytmów
├── launch/
│   └── system.launch.py   ← Launch file ROS2
├── test/
│   ├── unit/              ← Testy jednostkowe
│   └── integration/       ← Testy integracyjne
└── scripts/
    ├── run_demo.sh        ← Skrypt demonstracyjny
    └── collect_data.py    ← Zbieranie danych
```

### Najlepsze praktyki

**Wersjonowanie (Git):**
```bash
# Rozgałęzienia dla funkcjonalności
git checkout -b feature/object-detection
git checkout -b feature/grasping
git checkout -b bugfix/ik-solver

# Częste commity z opisami
git commit -m "Add: inverse kinematics solver for G1 arm"
git commit -m "Fix: joint limits checking"
```

**Testy:**
```bash
# Pisz testy dla krytycznych funkcji
colcon test --packages-select my_g1_project
```

**Dokumentacja:**
```cpp
/**
 * @brief Oblicza kinematykę odwrotną dla ramienia G1
 * 
 * @param target_pos Docelowa pozycja [x, y, z] w metrach
 * @param joint_angles [out] Wynikowe kąty stawów [rad]
 * @return true jeśli rozwiązanie znalezione, false w przeciwnym razie
 * 
 * @note Funkcja używa algorytmu Levenberg-Marquardt
 * @warning Sprawdź limity stawów przed wysłaniem do robota
 */
bool inverse_kinematics(const Vector3d& target_pos, 
                       std::vector<double>& joint_angles);
```

### Zasoby pomocnicze

**Dokumentacja:**
- ROS2 Documentation: https://docs.ros.org/
- Unitree Support: https://support.unitree.com/
- MoveIt2: https://moveit.picknik.ai/

**Kursy online:**
- ROS2 Tutorials (oficjalne)
- Gazebo Simulation
- Computer Vision (OpenCV, PCL)

**Społeczności:**
- ROS Discourse: https://discourse.ros.org/
- Robotics Stack Exchange
- GitHub Issues w unitree_ros2

---

## Podsumowanie

**Wybór projektu:**
- 🟢 **Początkujący:** Projekty 1-3
- 🟡 **Średnio-zaawansowani:** Projekty 4-6
- 🔴 **Zaawansowani:** Projekty 7-10

**Kluczowe umiejętności:**
- ✅ Programowanie w C++ i Python
- ✅ Znajomość ROS2
- ✅ Podstawy kinematyki i dynamiki
- ✅ Podstawy computer vision (dla projektów z percepcją)
- ✅ Podstawy machine learning (dla projektów z ML)

**Pamiętaj:**
- 📚 Czytaj dokumentację
- 🧪 Testuj często
- 💾 Wersjonuj kod
- 🤝 Współpracuj z innymi
- 🎯 Zacznij od małych celów
- 🔐 Bezpieczeństwo przede wszystkim!

---

**Powodzenia w realizacji projektów! 🤖🎓🚀**
