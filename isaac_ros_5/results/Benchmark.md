# Benchmark cuVSLAM — stereo IR (cam2)

Misura delle prestazioni di cuVSLAM (Isaac ROS 5.0, cuVSLAM 15.0) sulla Jetson Orin Nano,
eseguito sulle registrazioni (bag) delle D455, e estrazione delle traiettorie stimate.

## Cosa misura

Per ogni registrazione:

- **velocità**: tempo di elaborazione di ogni frame (media e massimo);
- **carico**: CPU (sistema e solo cuVSLAM), GPU e RAM durante l'elaborazione;
- **frame persi**: in registrazione, in elaborazione, e frame con tracking perso;
- **traiettoria**: posizione e rotazione per ogni frame, in CSV.

## Configurazione del test

| Voce | Valore |
|---|---|
| Hardware | Jetson Orin Nano, modalità MAXN_SUPER, `jetson_clocks` attivo |
| Software | Isaac ROS 5.0 (ROS 2 Lyrical), cuVSLAM 15.0, container Docker |
| Camera | cam2 (frontale), `infra1` + `infra2` (stereo IR) |
| Launch | `launch/cuvslam_stereo_cam2_bench.launch.py` (visualizzazione disattivata) |
| Riproduzione | `ros2 bag play` a velocità reale (`--rate 1.0`), solo i topic IR di cam2 e `/tf_static` |
| Registrazioni | 17 bag con stereo IR completo (`scenario_100/101` esclusi: manca `infra2`) |

La riproduzione a velocità reale è voluta: cuVSLAM riceve i frame con lo stesso ritmo
della registrazione, come se fossero dal vivo. Il carico di CPU e GPU è quindi quello
di un uso reale (con le differenze elencate in [Limiti](#limiti-noti)).

## File 

```
results/
  summary.csv               una riga per bag, metriche principali
  summary.md                le stesse, come tabella Markdown
  summary_detailed.csv      una riga per bag, tutte le metriche e i conteggi
  <bag>/
    frames.csv              una riga per frame elaborato (tempi, CPU, GPU, RAM)
    odometry.csv            traiettoria dell'odometria visiva, una posa per frame
    slam_odometry.csv       traiettoria SLAM calcolata durante l'elaborazione
    slam_poses.csv          traiettoria SLAM finale, ottimizzata dopo le chiusure d'anello
    cuvslam.log, monitor.log, play.log
```

### `frames.csv` (una riga per frame)

| Colonna | Significato |
|---|---|
| `timestamp` | timestamp del frame (s, epoch), uguale a quello delle immagini nel bag |
| `window_ms` | durata della finestra di misura: tempo dal frame precedente |
| `vo_state` | stato del tracking: 1 = riuscito, 2 = perso |
| `node_callback_ms` | tempo totale del nodo cuVSLAM per il frame |
| `track_ms` | tempo del solo algoritmo di tracking |
| `cpu_system_pct` | CPU di tutto il sistema nella finestra (% dei 6 core) |
| `cpu_cuvslam_pct` | CPU del solo processo cuVSLAM nella finestra (% dei 6 core) |
| `gpu_pct` | GPU nella finestra (%) |
| `ram_used_mb` | RAM usata da tutto il sistema (MB) |

### File delle pose (`odometry.csv`, `slam_odometry.csv`, `slam_poses.csv`)

| Colonna | Significato |
|---|---|
| `timestamp` | timestamp del frame (s, epoch) |
| `x`, `y`, `z` | posizione (m) |
| `qx`, `qy`, `qz`, `qw` | rotazione come quaternione |
| `roll_deg`, `pitch_deg`, `yaw_deg` | la stessa rotazione in angoli (gradi) |

Le pose sono quelle del frame `cam2_link` (convenzione ROS: x in avanti, y a sinistra,
z in alto) e sono **relative al punto di partenza**, che vale (0, 0, 0) con rotazione nulla.

- `odometry.csv`: odometria visiva. Frame per frame, ma accumula deriva.
- `slam_odometry.csv`: posa SLAM com'era al momento in cui il frame è stato elaborato.
- `slam_poses.csv`: posa SLAM finale, corretta dalle chiusure d'anello. È quella da usare
  per il confronto con le planimetrie.

Le pose esistono solo per i frame con tracking riuscito (`vo_state = 1`).

## Le metriche (`summary.csv`)

### Frame

| Colonna | Cosa indica | Come si calcola |
|---|---|---|
| `duration_s` | durata della registrazione | dal primo all'ultimo messaggio IR, con l'orario in cui il bag li ha registrati |
| `camera_fps` | fps a cui la camera ha prodotto i frame IR | (frame della camera − 1) / `duration_s` |
| `processed_fps` | fps elaborati da cuVSLAM | `camera_fps` × frame elaborati / frame della camera |
| `lost_recording_%` | frame prodotti dalla camera ma senza coppia stereo completa nel bag | 1 − coppie nel bag / frame della camera |
| `lost_processing_%` | coppie stereo presenti nel bag ma non elaborate da cuVSLAM | 1 − frame elaborati / coppie nel bag |
| `tracking_lost_%` | frame elaborati in cui cuVSLAM ha perso il tracking | frame con `vo_state ≠ 1` / frame elaborati |

**Frame della camera**: un frame esiste se nel bag c'è almeno uno dei suoi quattro messaggi
(`camera_info` e immagine, sinistra e destra), con timestamp entro 5 ms. Così un messaggio
perso in registrazione non fa sparire il frame dal conteggio.

**Coppia stereo**: immagine sinistra e destra con timestamp entro 5 ms, la stessa soglia
che usa cuVSLAM (`sync_matching_threshold_ms`).

### Tempi di elaborazione

| Colonna | Cosa indica | Come si calcola |
|---|---|---|
| `time_mean_ms` | tempo medio di elaborazione di un frame | media di `node_callback_ms` |
| `time_max_ms` | tempo massimo di elaborazione di un frame | massimo di `node_callback_ms` |

Il tempo viene da cuVSLAM stesso (messaggio `visual_slam/status`, un messaggio per frame).
Si usano **solo i frame con tracking riuscito**: quando il tracking fallisce cuVSLAM
termina prima, e quei frame abbasserebbero la media. Il tempo del solo algoritmo
(`track_ms`) è in `summary_detailed.csv`.

### Carico

| Colonna | Cosa indica |
|---|---|
| `cpu_cuvslam_mean_%`, `cpu_cuvslam_max_%` | CPU usata dal solo processo cuVSLAM |
| `cpu_system_mean_%`, `cpu_system_max_%` | CPU di tutto il sistema, compresa la lettura del bag |
| `gpu_mean_%`, `gpu_max_%` | GPU |
| `ram_max_mb` | RAM massima usata da tutto il sistema |

Le percentuali di CPU sono riferite a **tutti i 6 core**: 100% = tutti i core occupati.

**Come si misurano.** Ogni volta che cuVSLAM finisce un frame, il monitor legge i contatori
di CPU di Linux (`/proc/stat` e quelli del processo cuVSLAM). La differenza con la lettura
precedente dà il carico nella **finestra** tra i due frame. La GPU viene letta ogni 10 ms
(`/sys/devices/platform/bus@0/17000000.gpu/load`), e per ogni frame si fa la media dei
campioni che cadono nella sua finestra.

**Media**: media pesata sulla durata delle finestre, cioè il carico medio nel tempo.

**Massimo**: il valore più alto della media calcolata su **intervalli di 1 secondo**.
Non si usa il massimo del singolo frame, perché Linux conta il tempo di CPU a scatti di
10 ms: in una finestra di pochi millisecondi un solo scatto vale il 50–100% e falserebbe
il valore.

### Altre colonne in `summary_detailed.csv`

`camera_frames`, `recorded_pairs`, `processed_frames` (conteggi), `timestamp_jumps`
(salti di oltre 1 s nei timestamp delle immagini, vedi [Limiti](#limiti-noti)), `recorded_fps`
(coppie nel bag in fps), `max_gap_ms` (intervallo più lungo tra due frame elaborati),
`track_ms_mean` / `track_ms_max` (tempo del solo tracking).

## Come leggere i numeri

- **Tempo reale**: cuVSLAM regge il tempo reale se `time_max_ms` resta sotto l'intervallo
  tra due frame, cioè 1000 / `camera_fps` (66 ms a 15 fps, 33 ms a 30 fps).
- **Margine sulla Jetson**: `cpu_cuvslam_mean_%` e `gpu_mean_%` indicano quanta capacità
  usa lo SLAM, e quindi quanta ne resta per il resto della pipeline.
- **Robustezza**: `tracking_lost_%` alto indica scene difficili per lo stereo IR
  (per esempio esposizione alla luce o poche texture).
