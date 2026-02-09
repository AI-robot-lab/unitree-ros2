/**
 * Ten przykład demonstruje jak używać ROS2 do odbierania niskopoziomowych stanów 
 * robota Unitree Go2.
 * 
 * FUNKCJA: Program subskrybuje topik ze stanem niskopoziomowym (LowState) i wyświetla:
 *          - Dane z IMU (kąty Eulera, kwaternion, żyroskop, akcelerometr)
 *          - Stany wszystkich silników (pozycje, prędkości, momenty, temperatury)
 *          - Siły w stopach (pomiary i estymacje)
 *          - Stan baterii (napięcie, prąd)
 * 
 * CEL EDUKACYJNY: Zrozumienie niskopoziomowej struktury danych robota,
 *                 dostęp do szczegółowych informacji sensorycznych,
 *                 monitorowanie stanu technicznego robota.
 * 
 * RÓŻNICA od read_motion_state: Ten program pokazuje "surowe" dane z czujników
 *                                i silników, podczas gdy SportModeState pokazuje
 *                                dane przetworzone i gotowe do nawigacji.
 * 
 * This example demonstrates how to use ROS2 to receive low states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

// ===== KONFIGURACJA - CO WYŚWIETLAĆ =====
constexpr bool INFO_IMU = true;         
// true = wyświetlaj dane z IMU (czujnika inercyjnego)
// IMU mierzy orientację, prędkości kątowe i przyspieszenia
// Set 1 to info IMU states

constexpr bool INFO_MOTOR = true;
// true = wyświetlaj stany wszystkich 12 silników
// Przydatne do diagnostyki i monitorowania pracy silników
// Set 1 to info motor states

constexpr bool INFO_FOOT_FORCE = true;
// true = wyświetlaj siły działające na stopy
// Pomaga zrozumieć rozkład obciążenia podczas chodu
// Set 1 to info foot force states

constexpr bool INFO_BATTERY = true;
// true = wyświetlaj stan baterii (napięcie, prąd)
// Ważne dla monitorowania czasu pracy robota
// Set 1 to info battery states

constexpr bool HIGH_FREQ = true;
// true = subskrybuj dane z wysoką częstotliwością (500Hz)
// false = niska częstotliwość (20Hz)
// UWAGA: Wysoką częstotliwość używaj gdy potrzebujesz precyzyjnego
//        pomiaru dynamiki (np. do sterowania niskopoziomowego)
// Set 1 to subscribe to low states with high frequencies (500Hz)

// ===== KLASA GŁÓWNA PROGRAMU =====
class LowStateSuber : public rclcpp::Node {
 public:
  // KONSTRUKTOR
  LowStateSuber() : Node("low_state_suber") {
    // --- Wybór topiku w zależności od wymaganej częstotliwości ---
    // "hf/lowstate" - błąd w oryginalnym kodzie, powinno być "lf/lowstate"
    // "lowstate" - wysoka częstotliwość (500 Hz)
    // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies)
    // topic
    const auto *topic_name = "hf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }
    
    // --- Utworzenie subskrybenta ---
    // Podobnie jak w read_motion_state, ale dla innego typu wiadomości (LowState)
    // The suber  callback function is bind to low_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_go::msg::LowState>(
        topic_name, 10,
        [this](const unitree_go::msg::LowState::SharedPtr data) {
          topic_callback(data);
        });
  }

 private:
  // ===== FUNKCJA CALLBACK - PRZETWARZANIE DANYCH =====
  void topic_callback(const unitree_go::msg::LowState::SharedPtr &data) {
    
    // --- BLOK 1: DANE Z IMU (INERTIAL MEASUREMENT UNIT) ---
    if (INFO_IMU) {
      // IMU to czujnik inercyjny - mierzy orientację i ruch robota
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      imu_ = data->imu_state;

      // --- Kąty Eulera (Roll, Pitch, Yaw) ---
      // RPY to intuicyjny sposób opisywania orientacji:
      // Roll (rpy[0]) - przechylenie na boki (obrót wokół osi X)
      // Pitch (rpy[1]) - przechylenie przód-tył (obrót wokół osi Y)  
      // Yaw (rpy[2]) - obrót w poziomie (obrót wokół osi Z)
      // Wartości w RADIANACH (nie stopniach!)
      // Konwencja ZYX: najpierw Yaw, potem Pitch, na końcu Roll
      RCLCPP_INFO(this->get_logger(),
                  "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                  imu_.rpy[1], imu_.rpy[2]);
      
      // --- Kwaternion (alternatywna reprezentacja orientacji) ---
      // Kwaterniony nie mają problemu "gimbal lock" jak kąty Eulera
      // Format: [w, x, y, z] gdzie w to część skalarna
      // Używane w zaawansowanych algorytmach sterowania
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                  imu_.quaternion[3]);
      
      // --- Żyroskop (prędkości kątowe) ---
      // Mierzy jak szybko robot się obraca wokół każdej osi
      // Jednostki: rad/s (radiany na sekundę)
      // wx, wy, wz - prędkości kątowe wokół osi X, Y, Z
      RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
      
      // --- Akcelerometr (przyspieszenia liniowe) ---
      // Mierzy przyspieszenie liniowe (włącznie z grawitacją!)
      // Jednostki: m/s² (metry na sekundę kwadrat)
      // Gdy robot stoi nieruchomo, az ≈ 9.81 m/s² (grawitacja)
      RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu_.accelerometer[0], imu_.accelerometer[1],
                  imu_.accelerometer[2]);
    }

    // --- BLOK 2: STANY SILNIKÓW ---
    if (INFO_MOTOR) {
      // Robot Go2 ma 12 silników (po 3 na każdą z 4 nóg)
      // Układ silników na nodze: biodro (hip), kolano (knee), stopa (foot)
      // Numeracja: 0-2 (prawa przednia), 3-5 (lewa przednia), 
      //            6-8 (prawa tylna), 9-11 (lewa tylna)
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque

      for (int i = 0; i < 12; i++) {
        motor_[i] = data->motor_state[i];
        
        // Dla każdego silnika wyświetlamy:
        // - num: numer silnika (0-11)
        // - q: aktualna pozycja kątowa [radiany]
        // - dq: prędkość kątowa [rad/s]
        // - ddq: przyspieszenie kątowe [rad/s²]
        // - tau: oszacowany moment obrotowy [N⋅m] (Newton-metry)
        //   tau_est jest estymacją momentu zewnętrznego działającego na staw
        //   Przydatne do wykrywania kolizji lub przeszkód
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                    motor_[i].tau_est);
      }
    }

    // --- BLOK 3: SIŁY W STOPACH ---
    if (INFO_FOOT_FORCE) {
      // Robot Go2 ma czujniki siły w stopach
      // foot_force: pomiar bezpośredni z czujnika (wartość surowa, int16)
      // foot_force_est: estymacja siły z modelu dynamicznego
      // Wartości NIE są w Newtonach - to surowe wartości z czujników!
      // Info foot force value (int not true value)
      for (int i = 0; i < 4; i++) {
        foot_force_[i] = data->foot_force[i];
        foot_force_est_[i] = data->foot_force_est[i];
      }

      // Wyświetlamy siły dla wszystkich 4 stóp:
      // foot0: prawa przednia, foot1: lewa przednia
      // foot2: prawa tylna, foot3: lewa tylna
      // Wyższe wartości = większa siła = większe obciążenie stopy
      RCLCPP_INFO(this->get_logger(),
                  "Foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force_[0], foot_force_[1], foot_force_[2],
                  foot_force_[3]);
      RCLCPP_INFO(
          this->get_logger(),
          "Estimated foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
          foot_force_est_[0], foot_force_est_[1], foot_force_est_[2],
          foot_force_est_[3]);
    }

    // --- BLOK 4: STAN BATERII ---
    if (INFO_BATTERY) {
      // Monitorowanie baterii jest kluczowe dla bezpiecznej pracy!
      // Info battery states
      // battery current
      // battery voltage
      battery_current_ = data->power_a;  // Prąd [Ampery]
      battery_voltage_ = data->power_v;  // Napięcie [Wolty]

      // UWAGA: 
      // - Napięcie spada gdy bateria się rozładowuje
      // - Prąd jest dodatni gdy robot pobiera energię (zazwyczaj)
      // - Gdy voltage < ~20V, bateria jest prawie pusta (dla typowej baterii)
      RCLCPP_INFO(this->get_logger(),
                  "Battery state -- current: %f; voltage: %f", battery_current_,
                  battery_voltage_);
    }
  }

  // ===== ZMIENNE CZŁONKOWSKIE KLASY =====
  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr suber_;
  // Wskaźnik do subskrybenta

  unitree_go::msg::IMUState imu_;          // Dane z IMU (przechowywane dla dalszego użycia)
  unitree_go::msg::MotorState motor_[12];  // Stany wszystkich 12 silników
  
  int16_t foot_force_[4]{};                // Siły w stopach - pomiar bezpośredni
  int16_t foot_force_est_[4]{};            // Siły w stopach - estymacja
  
  float battery_voltage_{};  // Napięcie baterii [V]
  float battery_current_{};  // Prąd baterii [A]
};

// ===== FUNKCJA MAIN =====
int main(int argc, char *argv[]) {
  // Inicjalizacja ROS2
  rclcpp::init(argc, argv);  // Initialize rclcpp
  
  // Uruchomienie noda - rozpoczyna nasłuchiwanie na topiki
  // Run ROS2 node which is make share with low_state_suber class
  rclcpp::spin(std::make_shared<LowStateSuber>());
  
  // Zamknięcie po Ctrl+C
  rclcpp::shutdown();
  return 0;
}