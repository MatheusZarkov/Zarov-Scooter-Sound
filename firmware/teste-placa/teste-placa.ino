// Primeiro teste da placa, sem nenhum fio ligado alem do USB.
// Mostra no monitor serial (115200) que o ESP32 esta vivo e os dados do chip.
// Se a placa tiver LED no GPIO2, ele pisca junto. A DevKitC costuma nao ter.

const int LED = 2;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32 vivo ===");
  Serial.printf("Chip: %s, revisao %d, %d nucleos, %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("Flash: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("RAM livre: %u KB\n", ESP.getFreeHeap() / 1024);
  Serial.printf("MAC: %012llX\n", ESP.getEfuseMac());
}

void loop() {
  static uint32_t n = 0;
  digitalWrite(LED, n % 2);
  Serial.printf("ligado ha %lu s\n", (unsigned long)(millis() / 1000));
  n++;
  delay(1000);
}
