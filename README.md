# SP-FT-180 · Yıldız-Poligon Tam Toroid

**Uygulama:** https://cellular1973-ai.github.io/sp-ft-180/sp_ft_180.html
(kök adres ve `spft180.html` aynı sayfayı açar)

SP-FT-180, tek telle Rodin (yıldız-poligon) düzeninde sarılmış Ø180 mm'lik bir tam toroid bobindir. Bobin beyaz kaidede durur; uçları kaidenin ön yüzündeki iki banana direğine bağlıdır. Sürücü ayrı bir kutudadır (12 V adaptör) ve kaideye iki kabloyla bağlanır. Telefon veya bilgisayardan Web Bluetooth ile kontrol edilir.

> Tıbbi cihaz değildir; herhangi bir hastalığın tanı veya tedavisi için tasarlanmamıştır. Kalp pili, defibrilatör veya başka bir aktif implant taşıyanlar kullanmamalıdır.

## Bobin

| | Değer |
|---|---|
| Gövde | Ø180 mm dış, Ø40 mm delik, 83 mm yükseklik, elips kesit |
| Sargı | tek tel, Rodin N = 37 poloidal × M = 15 toroidal, Ø1 mm bakır |
| Tel | 10,63 m · 233 mΩ · 42,3 µH (τ 0,18 ms) |
| Merkezde alan | 136 µT/A |
| Eksende +50 mm / +100 mm | 77 / 24 µT/A |
| Tüp içinde | 143 µT/A |
| 1 m uzakta | 0,03 µT/A |

Değerler Biot-Savart / Neumann hesabındandır; ölçümle doğrulanmadı.

## Uygulama (`sp_ft_180.html`)

- Bluetooth bağlantısı (cihaz adı `SPFT180`), Demo modu (cihazsız deneme)
- Sinyal: sinüs, darbe (%10–50 doluluk), 7,83 Hz dar darbe; frekans 0,5–200 Hz
- Şiddet: 100–3000 mA tepe
- Oturum: 10 / 20 / 30 / 60 dk veya en fazla 12 adımlı program (hazır programlar + kendi adımlarınız)
- Durum: bobin akımı, köprü gerilimi, besleme gerilimi, akım fazı, kalan süre, tahmini merkez ve tüp içi alan, dB/dt
- Telefona kurulabilir (PWA), çevrimdışı açılır

Web Bluetooth: Android'de Chrome, iPhone'da Bluefy tarayıcısı; sayfa HTTPS üzerinden açılmalıdır.

## Sürücü kutusu

- ESP32 DevKitC V4 (38 pin, WROOM-32), soketli
- 12 V DC adaptör (5,5/2,1 mm jak) → 2 A PTC sigorta → SS54 ters kutup diyotu
- Tam köprü: 2 × IR2104 + 4 × IRF540N (TO-220), 40 kHz PWM, tek kutuplu sürüş
- 47 µH seri şok, 10 mΩ şönt + INA240A1 akım ölçümü; genlik kapalı çevrimle ayarlanır
- AP63205 5 V regülatör (DevKitC beslemesi)
- Korumalar: 4,5 A anlık akım kesme, besleme 10–15 V dışında durma, bobin bağlı değil algısı

Kart 115 × 50 mm, 2 katman. Kutu 121 × 61 × 32 mm (3B baskı); kaide 124 × 90 × 38 mm.

## Firmware (`firmware/spft180_surucu/`)

ESP32 DevKitC, Arduino-ESP32 core 3.x. Taslaktır, donanımda henüz denenmedi.

```bash
arduino-cli compile -b "esp32:esp32:esp32:PartitionScheme=huge_app" --upload -p COMx firmware/spft180_surucu
```

Komutlar (seri port 115200 ve Bluetooth aynı):

```
on | off | a <mA> | t <dakika> | f <Hz> | m sinus | m darbe | d <yüzde> | w <us>
schumann | led on|off | s | cal
p <f>/<s|dNN|wNNN>/<dk>,...     örnek: p 7.83/w500/5,10/s/10,100/d25/5
```

Bluetooth servisi `7a1c0201-5b4e-4f6a-9c1d-2e3f4a5b6c7d` (komut `…0202`, durum `…0203`).
