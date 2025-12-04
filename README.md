# Log Analiz Yazılımı Teknik Dokümantasyonu

Bu proje, İHA görevlerinde uçuş kayıtlarının (.tlog) otonom olarak analiz edilmesi, kural ihlallerinin tespiti ve uçuş verilerinin görselleştirilmesi amacıyla C++ dilinde geliştirilmiştir.

Bu doküman, yazılımın mimarisini, algoritma mantığını ve kod yapısını açıklamaktadır.

## 1. Yazılım Mimarisi

Yazılım üç temel modülden oluşmaktadır:
1.  **Veri Ayıklama (Parsing):** Ham binary verinin anlamlı fiziksel büyüklüklere (GPS, İrtifa) çevrilmesi.
2.  **Durum Makinesi (State Machine):** Uçuşun fazlarının (Yerde bekleme, Tur atma, İhlal durumu) takibi.
3.  **Raporlama ve Görselleştirme:** Analiz sonuçlarının kullanıcıya sunulması.

## 2. Algoritma Mantığı

### 2.1. MAVLink Ayrıştırma (Parsing)
Kod, `mavlink_parse_char` fonksiyonunu kullanarak dosya akışındaki baytları okur. Ardupilot tarafından gönderilen `GLOBAL_POSITION_INT` (Msg ID: 33) paketlerini hedefler. Bu paketler seçilmiştir çünkü:
*   **time_boot_ms:** Zaman senkronizasyonu için.
*   **lat/lon:** 1E7 ölçeğinde hassas konum verisi için.
*   **relative_alt:** Kalkış noktasına göre irtifa (AGL) verisi için.

### 2.2. Geometrik Hesaplamalar (Haversine Formülü)
Dünya'nın küresel yapısı nedeniyle, iki GPS koordinatı arasındaki mesafe Öklid geometrisi yerine **Haversine Formülü** ile hesaplanmıştır. `get_distance_metres` fonksiyonu, enlem ve boylam farklarını kullanarak metre cinsinden hassas mesafe döndürür. Bu fonksiyon hem "Home" noktasına olan uzaklığı hem de "Yasak Bölgeye" (NFZ) olan mesafeyi hesaplamak için kullanılır.

### 2.3. Tur Algılama (Lap Detection Logic)
Uçuşun bütünsel analizi yerine, görev odaklı "Tur" analizi yapılmıştır. Bunun için bir Durum Makinesi (State Machine) tasarlanmıştır:

*   **Home Noktası Belirleme:** Drone 1 metre irtifanın üzerine ilk çıktığı andaki konumu "Home" olarak kaydeder.
*   **Tur Başlangıcı:** Drone, Home noktasından `HOME_RADIUS_TH` (15m) + Histerezis (5m) kadar uzaklaştığında `is_in_lap` durumu `true` olur.
*   **Tur Bitişi:** Drone tekrar `HOME_RADIUS_TH` (15m) çemberinin içine girdiğinde tur tamamlanmış sayılır ve sonuçlar kaydedilir.

### 2.4. İhlal Kontrolü
Tur devam ederken (is_in_lap == true) her veri noktası için anlık kontrol yapılır:
*   **İrtifa İhlali:** `dp.alt > MAX_ALTITUDE_LIMIT` (120m) koşulu sağlanırsa tur geçersiz sayılır.
*   **NFZ İhlali:** Drone'un konumu ile NFZ merkezi arasındaki mesafe, NFZ yarı çapından küçükse (`dist < NFZ_RADIUS`) ihlal tespit edilir.

## 3. Kod Yapısı ve Veri Tipleri

### Yapılar (Structs)
*   `DataPoint`: Ham GPS ve zaman verilerini tutar. Varsayılan değer ataması (Uniform Initialization) ile bellek güvenliği sağlanmıştır.
*   `LapInfo`: Her turun sonucunu (ID, Geçerlilik Durumu, Hata Nedeni, Max İrtifa) tutar.

### Sabitler (Constants)
Performans optimizasyonu için değişmeyen değerler `constexpr` olarak tanımlanmıştır. Bu sayede derleme zamanında (compile-time) değerler kodun içine gömülür.
*   `MAX_ALTITUDE_LIMIT`: 120.0 metre
*   `NFZ_LAT / LON`: Yasak bölge koordinatları
*   `NFZ_RADIUS`: 50.0 metre

## 4. Görselleştirme (Matplotlib-cpp)

C++ içerisinde grafik çizimi için Python'un Matplotlib kütüphanesi `matplotlib-cpp` wrapper'ı ile entegre edilmiştir.

*   **Backend Yönetimi:** Linux sistemlerde grafik penceresi hatalarını önlemek için `plt::backend("TkAgg")` komutu ile Tkinter arayüzü aktif edilmiştir.
*   **Figure 1 (İrtifa Analizi):** Zamana bağlı irtifa değişimini çizer ve 120m sınırını kırmızı kesikli çizgi ile gösterir.
*   **Figure 2 (Konum Analizi):** Kuş bakışı (2D) uçuş rotasını çizer. Yasak bölgeyi (NFZ) kırmızı bir çember olarak haritaya işler, böylece ihlaller görsel olarak doğrulanabilir.

## 5. Dosya Listesi

*   **main.cpp:** Tüm analiz mantığını içeren ana kaynak kodu.
*   **generate_mission_log.py:** Yazılımı test etmek amacıyla, içinde kasıtlı hatalar (Valid, Invalid Altitude, Invalid Zone) barındıran sentetik `.tlog` dosyası üreten Python scripti.
*   **mission.tlog:** Python scripti tarafından üretilen örnek binary log dosyası.