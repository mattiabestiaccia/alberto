# EXCITE Satellite AOCS Simulation

Questo progetto implementa una simulazione completa del sistema di controllo di assetto e orbita (AOCS) per il satellite **EXCITE** (un CubeSat 12U), utilizzando il framework [Basilisk](http://hanspeterschaub.info/basilisk).

La simulazione include modelli dinamici dettagliati, algoritmi di volo (FSW) avanzati e una logica di missione autonoma basata su macchina a stati finiti (FSM).

## Caratteristiche Principali

### Dinamica (Dynamics)

Il modello dinamico (`EXCITE_Dynamics.py`) simula l'ambiente spaziale e l'hardware del satellite:

- **Ambiente**: Gravità (Terra, Sole, Luna), Gradiente di gravità, Resistenza atmosferica (modello esponenziale), Pressione di radiazione solare (SRP), Disturbo magnetico residuo.
- **Attuatori**:
  - 4 Ruote di Reazione (CubeSpace CW0162) in configurazione piramidale.
  - 3 Magnetorquers (MTB) per desaturazione e detumbling.
  - Propulsore chimico H2O2 (UniPi IOD) per manovre orbitali.
  - 4 Pannelli solari dispiegabili.
- **Sensori**:
  - Star Tracker.
  - Sensori Solari Grossolani (CSS).
  - Magnetometro (TAM).
  - IMU (Giroscopi con bias drift).
  - GPS/GNSS (simulato via SimpleNav).

### Software di Volo (FSW)

Il software di volo (`EXCITE_Fsw.py`) implementa la logica di controllo e navigazione:

- **Navigazione**:
  - **QUEST**: Determinazione d'assetto statica (Vettori Sole + Mag).
  - **SMEKF**: Filtro di Kalman Esteso Moltiplicativo per la stima ottima dell'assetto (fusione di Star Tracker, IMU, QUEST).
- **Controllo**:
  - Anello esterno: **MRP Steering** (limitazione di velocità angolare).
  - Anello interno: **Rate Servo** (inseguimento di velocità).
  - Gestione momento angolare: Desaturazione ruote tramite MTB.
  - **B-dot Controller**: Per il detumbling iniziale.
- **Logica di Missione (FSM)**:
  - Transizioni autonome basate su eventi (es. fine detumbling, eclissi, orari di missione).
  - Gestione automatica delle eclissi (puntamento antenna verso GS).

### Fasi della Missione

Lo scenario (`EXCITE_scenario.py`) simula una missione di 24 ore con le seguenti fasi:

1.  **Detumbling**: Stabilizzazione iniziale con B-dot.
2.  **Deployment**: Dispiegamento pannelli solari.
3.  **Sun-Safe**: Puntamento pannelli verso il Sole.
4.  **GS Pointing**: Puntamento antenna S-band verso la Ground Station di Pisa.
5.  **Payload Mode A**: Esperimento ReconfANT (Puntamento asse X).
6.  **Payload Mode B**: Esperimento GPU IoT (Puntamento asse -X).
7.  **Imaging Mode**: Puntamento Nadir per acquisizione immagini (Camera IM200).

## Struttura dei File

- `EXCITE_scenario.py`: Script principale per eseguire la simulazione. Configura lo scenario, gestisce il loop di simulazione e genera i grafici.
- `EXCITE_Dynamics.py`: Configurazione del modello fisico e dinamico del satellite (BSKDynamicModels).
- `EXCITE_Fsw.py`: Configurazione degli algoritmi di bordo (BSKFswModels) e della macchina a stati (FSM).
- `EXCITE_Plotting.py`: Modulo per la generazione dei grafici di analisi (non incluso in questo elenco ma referenziato).
- `EXCITE_Analysis.py`: Modulo per l'analisi numerica dei risultati (non incluso in questo elenco ma referenziato).
- `SMEKF.c` / `questAttDet.c`: Implementazioni C degli algoritmi di navigazione (se presenti come moduli custom).

## Requisiti

- Python 3.x
- [Basilisk](http://hanspeterschaub.info/basilisk) framework
- NumPy, Matplotlib

## Esecuzione

Per avviare la simulazione:

```bash
python3 EXCITE_scenario.py
```

La simulazione genererà log in console relativi agli eventi di missione (transizioni di modo, eclissi, ecc.) e produrrà grafici delle performance AOCS al termine.
