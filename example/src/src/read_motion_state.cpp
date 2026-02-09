/**
 * Ten przykład demonstruje jak używać ROS2 do odbierania stanów ruchu robota Unitree Go2.
 * 
 * FUNKCJA: Program subskrybuje topik ze stanem trybu sportowego (SportModeState) 
 *          i wyświetla informacje o:
 *          - Pozycji robota w przestrzeni (odometria)
 *          - Prędkości ruchu (liniowe i kątowe)
 *          - Typie chodu (trot, run, itp.)
 *          - Pozycjach i prędkościach stóp względem korpusu
 * 
 * CEL EDUKACYJNY: Nauka podstawowej komunikacji ROS2 - subskrypcja topików
 *                 i przetwarzanie przychodzących danych sensorycznych.
 * 
 * This example demonstrates how to use ROS2 to receive motion states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

// ===== KONFIGURACJA PROGRAMU =====
constexpr bool INFO_FOOT_STATE = true;
// Ustaw na true aby wyświetlać dodatkowe informacje o stopach
// (pozycja i prędkość każdej stopy względem korpusu robota)
// Set 1 to info foot states (foot position and velocity in body frame)

constexpr bool HIGH_FREQ = false;
// Ustaw na true aby subskrybować dane z wysoką częstotliwością (500Hz)
// false = niska częstotliwość (20Hz) - wystarczająca do monitorowania
// UWAGA: Wysoka częstotliwość zużywa więcej zasobów CPU/sieci
// Set 1 to subscribe to low states with high frequencies (500Hz)

// ===== KLASA GŁÓWNA PROGRAMU =====
// Dziedziczy po rclcpp::Node, co czyni ją nodem ROS2
class MotionStateSuber : public rclcpp::Node {
 public:
  // KONSTRUKTOR - wywoływany przy tworzeniu noda
  MotionStateSuber() : Node("motion_state_suber") {
    // --- KROK 1: Wybór topiku do subskrypcji ---
    // Wybieramy nazwę topiku w zależności od wymaganej częstotliwości:
    // "lf/sportmodestate" = low frequency (20 Hz) - oszczędne, wystarczające
    // "sportmodestate" = high frequency (500 Hz) - dokładniejsze, wymaga więcej zasobów
    // the cmd_puber is set to subscribe "sportmodestate" or "lf/sportmodestate"
    // (low frequencies) topic
    const auto *topic_name = "lf/sportmodestate";
    if (HIGH_FREQ) {
      topic_name = "sportmodestate";
    }

    // --- KROK 2: Utworzenie subskrybenta ---
    // Subskrybent to obiekt, który "nasłuchuje" na wybranym topiku
    // Parametry:
    //   - topic_name: nazwa topiku do subskrypcji
    //   - 10: rozmiar kolejki wiadomości (bufor na wypadek opóźnień)
    //   - lambda function: funkcja wywoływana przy każdej nowej wiadomości
    // The suber  callback function is bind to
    // motion_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        topic_name, 10,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          topic_callback(data);  // Przekazujemy dane do funkcji callback
        });
  }

 private:
  // ===== FUNKCJA CALLBACK =====
  // Wywoływana automatycznie przy każdej nowej wiadomości z topiku
  // Parametr 'data' zawiera wszystkie informacje o stanie ruchu robota
  void topic_callback(const unitree_go::msg::SportModeState::SharedPtr &data) {
    // --- WYŚWIETLANIE INFORMACJI O CHODZIE ---
    // gait_type: typ chodu robota (0=idle, 1=trot, 2=run, 3=wspinaczka po schodach)
    // foot_raise_height: wysokość podnoszenia stopy podczas chodu
    // Info motion states
    // Gait type and foot raise height
    // Robot position (Odometry frame)
    // Robot velocity (Odometry frame)
    RCLCPP_INFO(this->get_logger(),
                "Gait state -- gait type: %d; raise height: %f",
                data->gait_type, data->foot_raise_height);
    
    // --- WYŚWIETLANIE POZYCJI ROBOTA ---
    // position[0,1,2]: współrzędne x, y, z w układzie odometrii [metry]
    // body_height: aktualna wysokość korpusu nad ziemią [metry]
    // UWAGA: Pozycja jest szacowana przez odometrię - może się kumulować błąd!
    RCLCPP_INFO(this->get_logger(),
                "Position -- x: %f; y: %f; z: %f; body height: %f",
                data->position[0], data->position[1], data->position[2],
                data->body_height);
    
    // --- WYŚWIETLANIE PRĘDKOŚCI ROBOTA ---
    // velocity[0,1,2]: prędkości liniowe vx, vy, vz [m/s]
    // yaw_speed: prędkość obrotu wokół osi pionowej [rad/s]
    // Te wartości są w układzie współrzędnych odometrii
    RCLCPP_INFO(this->get_logger(),
                "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
                data->velocity[0], data->velocity[1], data->velocity[2],
                data->yaw_speed);

    if (INFO_FOOT_STATE) {
      // --- WYŚWIETLANIE STANÓW STÓP (opcjonalne) ---
      // Kopiujemy dane o pozycjach i prędkościach stóp do lokalnych zmiennych
      // Dane są w układzie współrzędnych korpusu robota (body frame)
      // Info foot states (foot position and velocity in body frame)
      for (int i = 0; i < 12; i++) {
        foot_pos_[i] = data->foot_position_body[i];  // Pozycje: [x0,y0,z0, x1,y1,z1, ...]
        foot_vel_[i] = data->foot_speed_body[i];      // Prędkości: [vx0,vy0,vz0, ...]
      }

      // Wyświetlamy dane dla każdej stopy osobno
      // Stopa 0: prawa przednia, 1: lewa przednia, 2: prawa tylna, 3: lewa tylna
      // num: numer stopy (0-3)
      // x,y,z: pozycja stopy względem korpusu [metry]
      // vx,vy,vz: prędkość stopy względem korpusu [m/s]
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  0, foot_pos_[0], foot_pos_[1], foot_pos_[2], foot_vel_[0],
                  foot_vel_[1], foot_vel_[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  1, foot_pos_[3], foot_pos_[4], foot_pos_[5], foot_vel_[3],
                  foot_vel_[4], foot_vel_[5]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  2, foot_pos_[6], foot_pos_[7], foot_pos_[8], foot_vel_[6],
                  foot_vel_[7], foot_vel_[8]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  3, foot_pos_[9], foot_pos_[10], foot_pos_[11], foot_vel_[9],
                  foot_vel_[10], foot_vel_[11]);
    }
  }
  
  // ===== ZMIENNE CZŁONKOWSKIE KLASY =====
  // Create the suber to receive motion states of robot
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  // Wskaźnik do subskrybenta - zarządza nasłuchiwaniem na topik
  
  float foot_pos_[12]{};   // Tablica pozycji stóp: [x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3]
  float foot_vel_[12]{};   // Tablica prędkości stóp: [vx0,vy0,vz0, ...]
};

// ===== FUNKCJA MAIN - PUNKT WEJŚCIA PROGRAMU =====
int main(int argc, char *argv[]) {
  // --- KROK 1: Inicjalizacja ROS2 ---
  // Musi być wywołana przed użyciem jakichkolwiek funkcji ROS2
  // Przetwarza argumenty linii poleceń i konfiguruje system
  rclcpp::init(argc, argv);  // Initialize rclcpp
  
  // --- KROK 2: Uruchomienie noda ---
  // std::make_shared<MotionStateSuber>() tworzy nowy obiekt naszej klasy
  // rclcpp::spin() uruchamia pętlę główną ROS2 - program będzie działał
  // dopóki nie zostanie przerwany (Ctrl+C)
  // W pętli tej wywoływane są funkcje callback przy nadejściu wiadomości
  // Run ROS2 node which is make share with motion_state_suber class
  rclcpp::spin(std::make_shared<MotionStateSuber>());
  
  // --- KROK 3: Zamknięcie ROS2 ---
  // Wywołane po przerwaniu programu (Ctrl+C)
  // Zwalnia zasoby i kończy komunikację
  rclcpp::shutdown();
  return 0;
}