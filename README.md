# Modüler Otonom Takip ve Güdüm

Bu paket, sabit kanatlı bir İHA için otonom hedef takip ve güdüm algoritmalarını
test edilebilir modüllere ayırır. Sistem yalnızca gecikmeli telemetri ve hedef
bbox girdisiyle çalışacak şekilde tasarlanmıştır. Kamera burun kısmında sabit ve
gövde ileri ekseniyle hizalı kabul edilir; gimbal, attitude veya ek sensör verisi
kullanılmaz.

## Kurulum

Python 3.10 veya üstüyle, proje kök dizininde:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
```

Bağımlılıklar (`NumPy` ve `PyYAML`) `pyproject.toml` üzerinden kurulur.

## Belgeler

- [Mimari diyagramlar ve kod envanteri](docs/Otonom_Takip_Mimari_Diyagramlar.pdf)
- [Otonom takip ve güdüm teknik raporu](docs/Otonom_Takip_Baglantili_Aciklamali_Rapor.pdf)

## Modüller

- `target_selection`: Rakip telemetrilerinden 3B mesafeye göre hedef seçimi.
- `telemetry_compensation`: Gecikmeli telemetriyi hız-heading bilgisiyle ileri kestirme.
- `approach`: Görsel takip başlamadan önce telemetri tabanlı ilk yaklaşma.
- `vision_geometry`: Bbox piksel hatasını LOS açısına dönüştürme.
- `ekf`: LOS açısı ve açısal hızı için doğrusal Kalman filtresi.
- `pn_guidance`: LOS açısı ve açısal hızına dayanan, PN'den esinlenen yönelim komutu. Klasik PN uygulaması değildir.
- `speed_control`: Mesafe ve bbox boyutuna göre airspeed seçimi.
- `validation`: Bbox ve gecikmeli telemetri için veri kalite kontrolleri.
- `state_machine`: `APPROACH`, `TRACK`, `LOST`, `FAILSAFE` durum yönetimi.
- `safety`: Merkezi komut limitleri ve failsafe komutu.
- `command_smoothing`: Ani komut değişimlerini sınırlama.
- `logging`: JSONL uçuş/takip kaydı.
- `config`: YAML tabanlı sistem ayarları.
- `camera_calibration`: Kamera FOV/focal length doğrulama ve yükleme.
- `target_lock`: Hedef ID kilidi ve hedef değiştirme hysteresis'i.
- `replay`: JSONL log tekrar oynatma ve state özeti.
- `simulation`: Uzun sentetik takip senaryoları.
- `autopilot_adapter`: Gerçek bağlantı içermeyen otopilot adapter sınırı.
- `pipeline`: Tüm akışı birleştiren üst seviye sınıf.

## Çalıştırma

```bash
python3 -m examples.synthetic_scenario
python3 -m examples.long_synthetic_simulation
python3 -m unittest discover -v
```

JSONL kaydı varsa `python -m examples.replay_log yol/kayit.jsonl` ile yeniden işlenebilir.
Kayıt varsayılan olarak kapalıdır; `config/default.yaml` içinde `logging.path` ayarlanarak açılır.

Config dosyasıyla kullanım:

```python
from autonomous_tracking import AutonomousTrackingPipeline, PipelineConfig

config = PipelineConfig.from_file("config/default.yaml")
pipeline = AutonomousTrackingPipeline(config)
```

## Notlar

Tüm koordinat ve hız hesapları SI birimleriyle yapılır. Açı alanları model
dokümantasyonunda derece veya radyan olarak açıkça belirtilmiştir.
`AutonomousTrackingPipeline.process(...)` her çağrıda `TrackingStatus` döndürür;
asıl uçuş komutu `status.command` alanındadır.
Bbox hedef kimliği içermez; verilen kutunun seçilen telemetri hedefine ait olduğu
varsayılır. Gerçek MAVLink/PX4 gönderimi veya mod değişikliği yapılmaz.
`DryRunAutopilotAdapter` yalnızca hesaplanan komutu bellekte tutar.
