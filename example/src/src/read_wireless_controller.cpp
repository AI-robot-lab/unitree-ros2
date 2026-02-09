/**
 * Ten przykład demonstruje jak używać ROS2 do odbierania stanów bezprzewodowego 
 * kontrolera robota Unitree Go2.
 * 
 * FUNKCJA: Program subskrybuje topik z danymi kontrolera i wyświetla:
 *          - Wartości obu joysticków (lewy i prawy)
 *          - Stan przycisków (jako wartość liczbowa)
 * 
 * CEL EDUKACYJNY: Nauka przetwarzania sygnałów z kontrolera,
 *                 podstawa do implementacji teleopercji (zdalnego sterowania),
 *                 zrozumienie mapowania analogowych sygnałów.
 * 
 * ZASTOSOWANIE: Ten kod to fundament do budowy systemu teleoperacji - 
 *               możesz go rozszerzyć, aby kontroler sterował ruchem robota!
 * 
 * This example demonstrates how to use ROS2 to receive wireless controller
 * states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

// ===== KLASA GŁÓWNA PROGRAMU =====
// Odbiera i wyświetla dane z kontrolera bezprzewodowego Unitree
class WirelessControllerSuber : public rclcpp::Node {
 public:
  // KONSTRUKTOR
  WirelessControllerSuber() : Node("wireless_controller_suber") {
    // --- Utworzenie subskrybenta dla topiku kontrolera ---
    // Kontroler Unitree publikuje swój stan na topiku "/wirelesscontroller"
    // Częstotliwość: ~50 Hz (dostatecznie szybko dla ludzkiej reakcji)
    // the cmd_puber is set to subscribe "/wirelesscontroller" topic
    suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr data) {
          topic_callback(data);
        });
  }

 private:
  // ===== FUNKCJA CALLBACK - PRZETWARZANIE DANYCH Z KONTROLERA =====
  void topic_callback(
      const unitree_go::msg::WirelessController::SharedPtr& data) {
    // --- STRUKTURA DANYCH Z KONTROLERA ---
    // Kontroler Unitree ma układ podobny do kontrolera Xbox/PlayStation:
    //
    //  [L2]                    [R2]     ← Triggery (na górze)
    //  [L1]                    [R1]     ← Bumpers (na górze)
    //
    //   ╔═══╗                 (Y)       ← Przyciski
    //   ║ L ║            (X)  [A]  (B)  ← (oznaczenia jak Xbox)
    //   ╚═══╝                           
    //                        ╔═══╗
    //                        ║ R ║      ← Prawy joystick
    //                        ╚═══╝
    //
    // Lewy joystick (L):
    // - lx: oś X (lewo-prawo), zakres [-1.0, 1.0]
    //   * -1.0 = maksymalnie w lewo
    //   *  0.0 = centrum
    //   * +1.0 = maksymalnie w prawo
    // lx: Left joystick x value
    
    // - ly: oś Y (góra-dół), zakres [-1.0, 1.0]
    //   * +1.0 = maksymalnie do góry
    //   *  0.0 = centrum
    //   * -1.0 = maksymalnie w dół
    // ly: Left joystick y value
    
    // Prawy joystick (R):
    // - rx: oś X (lewo-prawo), zakres [-1.0, 1.0]
    // rx: Right joystick x value
    
    // - ry: oś Y (góra-dół), zakres [-1.0, 1.0]
    // ry: Right joystick y value
    
    // Przyciski:
    // - keys: wartość liczbowa reprezentująca stan wszystkich przycisków
    //   Każdy przycisk odpowiada jednemu bitowi w tej liczbie
    //   Przykład: keys = 1 (binarnie 0001) = przycisk A wciśnięty
    //            keys = 5 (binarnie 0101) = przyciski A i X wciśnięte
    // keys value

    // --- WYŚWIETLANIE WARTOŚCI ---
    RCLCPP_INFO(
        this->get_logger(),
        "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
        data->lx, data->ly, data->rx, data->ry, data->keys);
    
    // 💡 WSKAZÓWKA DLA STUDENTÓW:
    // Możesz rozszerzyć ten kod, aby:
    // 1. Dekodować poszczególne przyciski z wartości 'keys'
    //    Przykład: bool button_A = (data->keys & 0x01) != 0;
    //
    // 2. Implementować strefę martwą (deadzone) dla joysticków
    //    Przykład: if (abs(data->lx) < 0.1) lx = 0.0;
    //
    // 3. Mapować wartości joystick na komendy ruchu robota
    //    Przykład: vx = data->ly * MAX_SPEED;
    //
    // 4. Reagować na przyciski (zmiana trybu, zapisanie pozycji, etc.)
    //
    // Zobacz docs/PRZEWODNIK_STUDENTA.md dla pełnego przykładu!
  }

  // ===== ZMIENNE CZŁONKOWSKIE =====
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber_;
  // Wskaźnik do subskrybenta topiku kontrolera
};

// ===== FUNKCJA MAIN =====
int main(int argc, char* argv[]) {
  // Inicjalizacja ROS2
  rclcpp::init(argc, argv);  // Initialize rclcpp
  
  // Uruchomienie noda - rozpoczyna odbieranie danych z kontrolera
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<WirelessControllerSuber>());
  
  // Zamknięcie po Ctrl+C
  rclcpp::shutdown();
  return 0;
}