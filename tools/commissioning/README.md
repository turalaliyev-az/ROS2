# Robotun istismara verilmə (commissioning) alətləri

Mühərrik, odometriya və naviqasiya yoxlamaları üçün. ROS paketi deyil, colcon bu qovluğa
toxunmur. Qeydlər `logs/` qovluğuna yazılır.

| Alət | Servis | Nə edir |
|---|---|---|
| `verify_decoding.py <imp_sol> <imp_sağ>` | **dayandırılmalı** | sabit PWM ilə qısa dönmə; enkoder dekodlamasını giroskopla yoxlayır (sürət dövrəsi işə düşmədən) |
| `speed_step.py <m/s> [m]` | **dayandırılmalı** | ESP32-yə birbaşa sürət əmri, hər təkərin sürət profili ~100 Hz |
| `debug_step.py <m/s> [m]` | **dayandırılmalı** | eyni, üstəlik firmware tənzimləyicisinin daxili vəziyyəti (hədəf, ölçülən, PWM, təkan, inteqral) |
| `direction_check.py` | işləyir | 0.3 m irəli; "irəli"ni enkoder, kamera və lidarla yoxlayır |
| `ground_test.py rotate/straight ...` | işləyir | yerində dönmə / düz gediş, enkoder + giroskop + lidar |
| `calibrate_odometry.py [m]` | işləyir | lidar skan uyğunlaşdırması (ICP) ilə təkər diametri, iz, sol/sağ fərqi |
| `calibrate_odometry.py --offset` | işləyir | 2 × 180° dönmə ilə lidarın dönmə mərkəzinə görə yeri |
| `camera_check.py` | işləyir | kameranın yerini lidarla müqayisə (divara 1–3 m baxaraq) |
| `loc_check.py` | işləyir | robot xəritədə lokalizasiya olunubmu (skan divarlara düşürmü) |
| `global_localize.py [--dry-run]` | işləyir | robotun xəritədəki yerini yalnız skanla tapır və AMCL-ə verir (veb server bunu özü də edir, aşağıya bax) |
| `nav_test.py` | işləyir | qəbul testi: A (2–3 m), B (5–9 m), başlanğıca qayıtma, veb server üzərindən |
| `nav_monitor.py [dəq]` | işləyir | hədəfləri izləyir: nəticə, müddət, xəta, sürət, bərpa, collision monitor, nasazlıq |
| `firmware_sim/` | — | firmware-in real C sürət tənzimləyicisini sürtünmə modeli ilə kompüterdə sınayır |

"İşləyir" olan alətlər əmrləri collision monitor üzərindən göndərir, yəni lidar qoruması aktivdir.
"Dayandırılmalı" olanlar üçün `systemctl --user stop rover` lazımdır və **lidar qoruması yoxdur**:
robotun qarşısı və arxası boş olmalıdır.

## İki təkərli platforma: yekun (2026-10-16)

- **Profil:** `diff2`, platforma 0.50 × 0.45 m, təkər ⌀13.94 sm (ICP), iz 0.555 m, kinematik
  iz 0.5443 m (`turn_slip_factor` 0.9807), 148/146 impuls/dövr (4×), ötürücü 180° fırladılıb
  (`reversed: true`), lidar x +0.179 m, y −0.018 m (2 × 180° ilə ölçülüb).
- **Firmware** (`~/Documents/PlatformIO/Projects/ROS2_DRIVER`, nüsxələr `~/.rover/backups/firmware/`):
  - 4× kvadratur dekodlama;
  - hər təkər üçün feedforward (sol 215, sağ 270, yuxarı sürətdə 150 + 0.4·imp/s), güclü P (2.0);
  - kiçik və yalnız hədəfə yaxın işləyən inteqral;
  - qopma təkanı və ilişərkən inteqralın dondurulması;
  - sürət aşma qoruması, PWM limiti 950;
  - kalibrləmə (`0xCC 0x35`) və diaqnostika (`0xCC 0x36`) çərçivələri.
- **Yerdə sürət izləmə:** 0.1 m/s → 0.10–0.11, 0.15 → 0.16–0.19, 0.3 → 0.33–0.36 m/s, ilişmə yoxdur,
  0.15 m/s-dən 2 sm-də dayanır.
- **Proqram:**
  - motor_bridge ötürücü nasazlığı monitoru (`/drive/fault`);
  - veb serverdə hədəf yoxlaması, tək thread-li executor (CPU ~90% → ~0%);
  - RPP `max_angular_accel` 3.0.
- **Naviqasiya qəbul testi:** 3/3 hədəf, mövqe xətası 6–12 sm.
- **Avtomatik lokalizasiya** (`rover_web/scan_match.py`): naviqasiya hər başlayanda veb server
  lidar skanını bütün xəritədə axtarır (~6 s, rejim "mövqe yoxlanılır…"). Saxlanmış mövqe yalnız
  bir-birinə oxşayan yerlər arasında seçim üçündür: köhnəlmiş mövqe yanlış yeri "təsdiqləyə"
  bilməz. "Mövqeyi tap" düyməsi də əvvəlcə skanla axtarır, tapmasa yerində dönür. Sınaq: saxlanmış
  mövqe 1.2 m / 40° səhv qoyuldu → düzgün yer tapıldı (2 sm, divara düşən 76%).
- **Servis:** `KillMode=mixed` (SIGINT yalnız `ros2 launch`-a; əvvəl düyünlər onu iki dəfə alıb
  təmizlənmə ortasında ölürdü).
- **Açıq:** kameranın yeri yoxlanmayıb (açıq ərazidə divar azdır). Bu, insan və masa aşkarlamasına
  təsir edir, naviqasiyaya yox.
