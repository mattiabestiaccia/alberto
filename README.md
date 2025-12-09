# EXCITE Satellite AOCS Simulation

Questo progetto implementa una simulazione completa del sistema di controllo di assetto e orbita (AOCS) per il satellite **EXCITE** (un CubeSat 12U), utilizzando il framework [Basilisk](http://hanspeterschaub.info/basilisk).

La simulazione include modelli dinamici dettagliati, algoritmi di volo (FSW) avanzati e una logica di missione autonoma basata su macchina a stati finiti (FSM).

## Caratteristiche Principali

### Dinamica (Dynamics)

Il modello dinamico (`excite/dynamics/spacecraft_model.py`) simula l'ambiente spaziale e l'hardware del satellite:

- **Ambiente**: Gravità (Terra, Sole, Luna), Gradiente di gravità, Resistenza atmosferica (modello esponenziale), Pressione di radiazione solare (SRP), Disturbo magnetico residuo.
- **Attuatori**:
  - 4 Ruote di Reazione (CubeSpace CW0162) in configurazione piramidale.
  - 3 Magnetorquers (MTB) per desaturazione e detumbling.
  - Propulsore chimico H2O2 (UniPi IOD) per manovre orbitali.
  - 4 Pannelli solari dispiegabili.
- **Sensori**:
  - Star Tracker (0.01° accuracy).
  - Sensori Solari Grossolani (CSS).
  - Magnetometro (TAM).
  - IMU Custom (Giroscopi con bias drift).
  - GPS/GNSS (simulato via SimpleNav).

### Software di Volo (FSW)

Il software di volo (`excite/fsw/fsw_model.py`) implementa la logica di controllo e navigazione:

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

Lo scenario (`excite/scenario/scenario.py`) simula una missione di 24 ore con le seguenti fasi:

1.  **Deployment**: Dispiegamento pannelli solari (60s).
2.  **Detumbling**: Stabilizzazione iniziale con B-dot (fino a 12h).
3.  **Sun-Safe**: Puntamento pannelli verso il Sole.
4.  **Initial Charge**: Ricarica batteria (1h).
5.  **Payload Mode A**: Esperimento ReconfANT (2h).
6.  **Payload Mode B**: Esperimento GPU IoT (1.5h).
7.  **GS Contact**: Puntamento antenna S-band verso Ground Station di Pisa.

## Struttura del Progetto

```
umb_v2/
├── README.md                       # Questo file
├── CLAUDE.md                       # Guida per Claude Code
├── LICENSE                         # MIT License
├── pyproject.toml                  # Build configuration (PEP 517/518)
│
├── excite/                         # Package Python principale
│   ├── __init__.py
│   ├── __main__.py                 # Entry point: python -m excite
│   │
│   ├── config/                     # Parametri configurazione
│   │   ├── spacecraft.py           # Massa, inerzia, geometria
│   │   ├── actuators.py            # RW, MTB, thruster specs
│   │   ├── sensors.py              # Star Tracker, IMU, TAM, CSS
│   │   ├── mission.py              # Orbita, timeline, IC
│   │   ├── control.py              # Guadagni controllori
│   │   ├── environment.py          # Gravità, atmosfera, SRP
│   │   └── constants.py            # Costanti fisiche
│   │
│   ├── dynamics/                   # Modelli dinamici
│   │   └── spacecraft_model.py     # BSKDynamicModels
│   │
│   ├── fsw/                        # Flight Software
│   │   └── fsw_model.py            # BSKFswModels + FSM
│   │
│   ├── scenario/                   # Scenario execution
│   │   └── scenario.py             # scenario_EXCITE class
│   │
│   ├── analysis/                   # Post-processing
│   │   └── plotting.py             # Grafici performance
│   │
│   ├── utilities/                  # Basilisk utilities
│   │   ├── BSK_masters.py          # BSKSim, BSKScenario base classes
│   │   └── BSK_Plotting.py         # Plot utilities
│   │
│   └── utils/                      # General utilities
│
├── excite_c_modules/               # Moduli C custom (disabilitati)
│   ├── src/
│   │   ├── SMEKF.c                 # Sequential Multiplicative EKF
│   │   └── questAttDet.c           # QUEST attitude determination
│   └── include/                    # Header files
│
├── scripts/                        # Script eseguibili
│   └── run_simulation.py           # Main CLI entry point
│
├── docs/                           # Documentazione tecnica
│   ├── 0_context.md                # Architettura e contesto
│   ├── SMEKF_Code.txt              # Note implementazione SMEKF
│   └── refactoring_plan.md         # Piano di riorganizzazione
│
├── old_codes/                      # File originali legacy (riferimento)
│   ├── EXCITE_scenario.py          # Scenario originale
│   ├── EXCITE_Dynamics.py          # Dynamics originale
│   ├── EXCITE_Fsw.py               # FSW originale
│   └── EXCITE_Plotting.py          # Plotting originale
│
├── tests/                          # Test suite
│   ├── conftest.py                 # Pytest configuration
│   └── test_simulation.py          # Simulation tests
│
└── data/                           # Output simulazione (non versionato)
    ├── plots/                      # Grafici generati
    └── telemetry/                  # Dati telemetrici
```

## Requisiti

### Prerequisiti

- Python 3.8+
- [Basilisk Framework](http://hanspeterschaub.info/basilisk) 2.0+
- CMake 3.18+ (per build moduli C)
- C compiler (gcc/clang)

### Installazione

1. **Clona il repository**:
```bash
git clone <repository-url>
cd umb_v2
```

2. **Installa dipendenze Python**:
```bash
pip install -r requirements.txt
```

3. **Build moduli C** (opzionale - se disponibili):
```bash
cd excite_c_modules
mkdir build && cd build
cmake ..
make
sudo make install  # Installa in Python site-packages
```

## Esecuzione

### Metodo 1: Script dedicato (consigliato)

```bash
# Simulazione completa 24 ore
python scripts/run_simulation.py

# Simulazione breve per test
python scripts/run_simulation.py --duration 2.0 --plots

# Con grafici
python scripts/run_simulation.py --plots
```

### Metodo 2: Entry point Python module

```bash
# Esecuzione standard
python -m excite

# Con opzioni
python -m excite --duration 6.0 --plots
```

### Metodo 3: Diretto (legacy)

```bash
# Usa ancora i vecchi file in simulation/
cd simulation
python3 EXCITE_scenario.py
```

## Output

La simulazione genera:
- **Console logs**: Eventi missione (transizioni FSM, eclissi, ecc.)
- **Grafici** (se abilitati): Performance AOCS
  - Attitude error (MRP)
  - Angular velocity
  - RW speeds
  - Control torques
  - Battery/Power
- **Telemetria** (futuro): File CSV in `data/telemetry/`

## Configurazione

Tutti i parametri sono centralizzati in `excite/config/`:

- **Modifica massa satellite**: Edit `excite/config/spacecraft.py` → `TOTAL_MASS_KG`
- **Tuning controllori**: Edit `excite/config/control.py` → `MRP_STEERING_K1`, `RATE_SERVO_P`, etc.
- **Cambio orbita**: Edit `excite/config/mission.py` → `ORBITAL_ELEMENTS`
- **Timeline missione**: Edit `excite/config/mission.py` → `MISSION_DURATION_HOURS`, durate fasi

## Documentazione

- **Architettura completa**: `docs/0_context.md`
- **Piano refactoring**: `docs/refactoring_plan.md`
- **Guida Claude Code**: `CLAUDE.md`
- **Basilisk Docs**: http://hanspeterschaub.info/basilisk

## Sviluppo

### Setup ambiente di sviluppo

```bash
pip install -r requirements-dev.txt
```

### Struttura modulare

Il progetto è organizzato come Python package con:
- Configurazioni separate dal codice
- Import espliciti da `excite.config.*`
- Compatibilità con Basilisk BSKSim/BSKScenario
- Build system per moduli C via CMake

## Stato Attuale

- ✅ Fase 1: Struttura base creata
- ✅ Fase 2: Configurazioni estratte (7 moduli config)
- ✅ Fase 3: Moduli Python migrati
- ⏳ Fase 4: Build C modules e testing
- ⏳ Fase 5: Documentazione completa

I file originali in `simulation/` sono preservati per riferimento fino al completamento della validazione.

## Licenza

MIT

## Autori

Alberto Bruschi - Università di Pisa
