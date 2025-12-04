import struct
import time
import math
import os

# MAVLink v2 protokolünü aktif et (Modern log formatı için)
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

# --- AYARLAR ---
FILENAME = "mission.tlog"
FREQUENCY = 10  # Hz (Saniyede kaç veri paketi yazılacak)

# Koordinatlar (ArduPilot SITL Varsayılan Konumu - CMAC)
HOME_LAT = -35.363261
HOME_LON = 149.165230

# Yasak Bölge (NFZ) Merkezi (Senaryo gereği kuzey-batıda)
NFZ_LAT = -35.362000
NFZ_LON = 149.164000

# Durum Değişkenleri (Global)
current_lat = HOME_LAT
current_lon = HOME_LON
current_alt = 0.0      # Relative Altitude (m)
t_boot_ms = 0          # Boot süresi (ms)

# Dosya ve Mavlink Nesnesi
f = None
mav = None

def init_log():
    global f, mav
    print(f"'{FILENAME}' dosyasi olusturuluyor...")
    f = open(FILENAME, 'wb')
    mav = mavutil.mavlink.MAVLink(None)

def close_log():
    global f
    f.close()
    print(f"Tamamlandi! '{FILENAME}' basariyla kaydedildi.")

def write_packet():
    """Mevcut konum verilerini dosyaya MAVLink paketi olarak yazar."""
    global t_boot_ms, f, mav
    
    # Verileri Tamsayıya Çevir (MAVLink standardı: lat/lon * 1E7, alt * 1000)
    lat_int = int(current_lat * 1e7)
    lon_int = int(current_lon * 1e7)
    alt_mm = int(current_alt * 1000)

    # GLOBAL_POSITION_INT (#33) Mesajı Oluştur
    msg = mavutil.mavlink.MAVLink_global_position_int_message(
        t_boot_ms,      # time_boot_ms
        lat_int,        # lat
        lon_int,        # lon
        alt_mm,         # alt (MSL - Deniz seviyesi, şu an relative ile aynı varsayıyoruz)
        alt_mm,         # relative_alt (Kalkış noktasına göre yükseklik - KRİTİK VERİ)
        0, 0, 0,        # vx, vy, vz (Hız verileri - şu an 0)
        4500            # hdg (Heading - Yön)
    )

    # Paketi Binary Hale Getir
    buf = msg.pack(mav)

    # .tlog Dosya Formatı: [64-bit Timestamp] + [MAVLink Packet]
    # Timestamp: Unix epoch (mikrosaniye cinsinden)
    timestamp_usec = int(time.time() * 1e6)
    
    # Big-Endian Unsigned Long Long (>Q) olarak zaman damgasını yaz
    f.write(struct.pack('>Q', timestamp_usec))
    # MAVLink paketini yaz
    f.write(buf)

def fly_to(target_lat, target_lon, target_alt, duration_sec):
    """Drone'u mevcut konumdan hedefe enterpolasyonla uçurur."""
    global current_lat, current_lon, current_alt, t_boot_ms
    
    steps = int(duration_sec * FREQUENCY)
    
    # Adım büyüklüklerini hesapla
    lat_step = (target_lat - current_lat) / steps
    lon_step = (target_lon - current_lon) / steps
    alt_step = (target_alt - current_alt) / steps
    
    for _ in range(steps):
        t_boot_ms += int(1000 / FREQUENCY)
        
        current_lat += lat_step
        current_lon += lon_step
        current_alt += alt_step
        
        write_packet()

def generate_scenario():
    init_log()
    
    # Kare Rota Boyutu (Derece cinsinden) ~200m
    OFFSET = 0.002 

    # --- BAŞLANGIÇ ---
    print("Simulasyon basliyor: Takeoff...")
    # 5 saniyede 40 metreye çık (Home noktasında)
    fly_to(HOME_LAT, HOME_LON, 40.0, 5)

    # ==========================
    # TUR 1: TEMİZ TUR (VALID)
    # ==========================
    print(">> TUR 1: Temiz tur atiliyor...")
    # 1. Köşe (Doğu)
    fly_to(HOME_LAT, HOME_LON + OFFSET, 40.0, 8)
    # 2. Köşe (Kuzey - Doğu)
    fly_to(HOME_LAT + OFFSET, HOME_LON + OFFSET, 40.0, 8)
    # 3. Köşe (Kuzey - Batı) - NFZ'ye girmeden dönüyor
    fly_to(HOME_LAT + OFFSET, HOME_LON, 40.0, 8)
    # 4. Eve Dönüş
    fly_to(HOME_LAT, HOME_LON, 40.0, 8)
    
    # Tur bitişi algılansın diye Home'da biraz bekle
    fly_to(HOME_LAT, HOME_LON, 40.0, 2)

    # ==========================
    # TUR 2: YÜKSEKLİK İHLALİ (INVALID ALTITUDE)
    # ==========================
    print(">> TUR 2: Yukseklik ihlali yapiliyor...")
    # 1. Köşe (Doğu)
    fly_to(HOME_LAT, HOME_LON + OFFSET, 40.0, 8)
    
    # 2. Köşeye giderken aniden yüksel! (135m - Limit 120m)
    fly_to(HOME_LAT + OFFSET, HOME_LON + OFFSET, 135.0, 8)
    
    # 3. Köşeye giderken yüksekte kal
    fly_to(HOME_LAT + OFFSET, HOME_LON, 130.0, 8)
    
    # Eve dönerken alçal
    fly_to(HOME_LAT, HOME_LON, 40.0, 8)
    fly_to(HOME_LAT, HOME_LON, 40.0, 2)

    # ==========================
    # TUR 3: NFZ İHLALİ (INVALID ZONE)
    # ==========================
    print(">> TUR 3: No-Fly Zone'a giriliyor...")
    
    # 1. Köşe (Doğu) - Sorun yok
    fly_to(HOME_LAT, HOME_LON + OFFSET, 40.0, 8)
    
    # 2. Köşe (Kuzey - Doğu) - Sorun yok
    fly_to(HOME_LAT + OFFSET, HOME_LON + OFFSET, 40.0, 8)
    
    # 3. Köşe İÇİN HEDEF DEĞİŞTİRİYORUZ!
    # Normalde (HOME_LAT + OFFSET, HOME_LON) gidecektik.
    # Şimdi NFZ'nin (HOME_LAT + 0.0012, HOME_LON - 0.0012) olduğu yere sürüyoruz.
    # Yasak bölgenin içinden geçecek.
    
    print("   -> NFZ rotasina sapiliyor!")
    fly_to(NFZ_LAT, NFZ_LON, 40.0, 10) # Tam NFZ merkezine git
    
    # Eve Dönüş
    fly_to(HOME_LAT, HOME_LON, 40.0, 10)
    
    # İniş (Land)
    print("Gorev bitti. İniş yapiliyor...")
    fly_to(HOME_LAT, HOME_LON, 0.0, 5)

    close_log()

if __name__ == "__main__":
    generate_scenario()
