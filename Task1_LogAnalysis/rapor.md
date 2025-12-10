
# Proje Süreç Raporu: İHA Görev Log Analiz ve Doğrulama Sistemi

## 1. Giriş ve Amaç
Bu projenin temel amacı, otonom hava araçlarının (İHA) gerçekleştirdiği uçuş görevlerinin,kurallara uygunluğunu denetleyen bir yazılım geliştirmektir. Manuel log incelemesi zaman alıcı ve hataya açık olduğundan, uçuş kayıtlarını (.tlog) otomatik olarak ayrıştıran, irtifa ve yasak bölge (NFZ) ihlallerini tespit eden ve sonuçları grafiksel olarak raporlayan bir C++ aracı tasarlanmıştır.

## 2. Kullanılan Yöntem ve Teknolojiler

Proje geliştirme sürecinde aşağıdaki teknoloji yığını ve yöntemler kullanılmıştır:

*   **Yazılım Dili:** C++17 (Performans ve tip güvenliği için).
*   **Protokol:** MAVLink v2 (Binary telemetri verisinin okunması için).
*   **Görselleştirme:** Matplotlib-cpp (Python'un grafik kütüphanesinin C++ entegrasyonu).
*   **Derleme Sistemi:** CMake.
*   **Geliştirme Ortamı:** Ubuntu 22.04 LTS / CLion.

## 3. Geliştirme Süreci ve Adımlar

### 3.1. Hazırlık ve Ortam Kurulumu
İlk aşamada geliştirme ortamı standardize edildi. Ubuntu 22.04 üzerinde gerekli derleyiciler (`g++`, `cmake`) ve Python kütüphaneleri (`python3-matplotlib`, `python3-tk`) kuruldu.
*   **Kütüphane Yönetimi:** Proje taşınabilirliğini artırmak için MAVLink ve Matplotlib-cpp başlık dosyaları (`.h`), proje klasörü yerine sistem kütüphane dizinine (`/usr/local/include`) taşındı. Bu sayede `CMakeLists.txt` dosyası sadeleştirildi.

### 3.2. Test Verisi Üretimi (Sentetik Log Oluşturma)
Analiz kodunu yazmadan önce, elimizde test edecek kontrollü bir veri seti yoktu. Rastgele bir log dosyası kullanmak yerine, senaryo tabanlı bir Python scripti (`generate_mission_log.py`) geliştirildi.
Bu script şunları simüle etti:
1.  **Tur 1:** Kurallara uygun temiz bir uçuş.
2.  **Tur 2:** 135 metreye çıkarak irtifa ihlali yapan bir uçuş.
3.  **Tur 3:** Koordinatlarını Yasak Bölge (NFZ) merkezine sürerek alan ihlali yapan bir uçuş.
*Sonuç:* MAVLink protokolüne tam uyumlu binary bir `.tlog` dosyası elde edildi.

### 3.3. C++ ile Veri Ayrıştırma (Parsing)
MAVLink protokolü binary (ikili) bir yapıdadır. Dosya `std::ifstream` ile bayt bayt okunarak `mavlink_parse_char` fonksiyonuna beslendi.
*   **Hedef Mesaj:** `GLOBAL_POSITION_INT` (ID: #33).
*   **Elde Edilen Veriler:** Zaman damgası (`time_boot_ms`), Enlem, Boylam ve Bağıl Yükseklik (`relative_alt`).

### 3.4. Algoritma Tasarımı: Tur Mantığı (Lap Logic)
Sadece "drone uçtu mu?" sorusu yerine "drone her turda kurallara uydu mu?" sorusuna odaklanıldı. Bunun için bir **Durum Makinesi (State Machine)** kuruldu:
*   **Home Noktası:** Drone 1m irtifaya ulaştığı anki konumu "Home" olarak kilitlendi.
*   **Tur Başlangıcı:** Drone Home noktasından 20 metre uzaklaştığında tur başladı olarak işaretlendi.
*   **Tur Bitişi:** Drone tekrar Home noktasının 15 metre yakınına geldiğinde tur tamamlandı ve rapor kaydedildi.

### 3.5. İhlal Kontrol Algoritmaları
*   **İrtifa Kontrolü:** Basit bir büyüktür operatörü ile `if (alt > 120.0)` kontrolü yapıldı.
*   **NFZ (Yasak Bölge) Kontrolü:** Dünya'nın küresel yapısı sebebiyle Öklid uzaklığı yerine **Haversine Formülü** kullanılarak iki GPS koordinatı arasındaki gerçek mesafe hesaplandı. Drone'un NFZ merkezine olan uzaklığı, NFZ yarıçapından küçükse ihlal sayıldı.

### 3.6. Görselleştirme
Sayısal verilerin anlaşılması zor olduğundan, 2 boyutlu grafikler eklendi:
1.  **Zaman - Yükseklik Grafiği:** 120m sınırını gösteren kırmızı bir referans çizgisi eklendi.
2.  **2D Rota Haritası:** Yasak bölgeyi temsil eden kırmızı bir çember çizdirildi. Drone'un bu çemberin içine girip girmediği görsel olarak kanıtlandı.

## 4. Karşılaşılan Zorluklar ve Çözümler

Süreç boyunca karşılaşılan teknik engeller ve uygulanan çözümler şöyledir:

| Zorluk | Açıklama | Çözüm |
| :--- | :--- | :--- |
| **Veri Eksikliği** | Elimizde kural ihlali içeren gerçek bir log dosyası yoktu. | Python ve Pymavlink kullanılarak özel senaryolu (Valid/Invalid turlar içeren) sentetik log üreten bir script yazıldı. |
| **Grafik Hataları** | C++ içinden grafik çizdirirken pencere açılmıyor veya "subplot failed" hatası veriyordu. | Matplotlib backend'i kod içinde `plt::backend("TkAgg")` olarak zorlandı ve sistem paketlerine `python3-tk` eklendi. |
| **Gürültü Önleme** | GPS verisindeki küçük oynamalar (Noise) drone yerde dururken tur başlatabiliyordu. | **Histerezis (Gecikme)** mantığı uygulandı. Turun başlaması için 20m uzaklaşma, bitmesi için 15m yaklaşma şartı koşuldu. |
| **Proje Yapısı** | Kütüphane dosyalarının proje klasöründe durması karmaşıklık yaratıyordu. | Başlık dosyaları `/usr/local/include` dizinine taşınarak Linux standartlarına uygun, temiz bir yapı kuruldu. |

## 5. Sonuç ve Değerlendirme

Geliştirilen **LogParser** yazılımı, üretilen test senaryoları üzerinde yapılan denemelerde başarı sağlamıştır.
*   Kurallara uyan turları "SUCCESS" olarak,
*   Yükseklik sınırını aşan turları "ALTITUDE VIOLATION" olarak,
*   Yasak bölgeye giren turları "NFZ VIOLATION" olarak doğru tespit etmiştir.

Bu çalışma, VTOL görevlerinde yer kontrol istasyonu operatörünün uçuş güvenliğini ve görev başarısını anlık veya uçuş sonrası analiz etmesi için güvenilir bir araç olarak kullanılabilir.
